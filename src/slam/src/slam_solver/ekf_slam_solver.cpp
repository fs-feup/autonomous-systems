#include "slam_solver/ekf_slam_solver.hpp"

EKFSLAMSolver::EKFSLAMSolver(const SLAMParameters& params,
                             std::shared_ptr<DataAssociationModel> data_association,
                             std::shared_ptr<V2PMotionModel> motion_model,
                             std::shared_ptr<LandmarkFilter> landmark_filter,
                             std::shared_ptr<std::vector<double>> execution_times,
                             std::shared_ptr<LoopClosure> loop_closure)
    : SLAMSolver(params, data_association, motion_model, landmark_filter, execution_times,
                 loop_closure),
      slam_parameters_(params) {
  this->covariance_ = Eigen::MatrixXd::Zero(3, 3);
  this->covariance_(0, 0) = params.pose_x_initial_noise_;
  this->covariance_(1, 1) = params.pose_y_initial_noise_;
  this->covariance_(2, 2) = params.pose_theta_initial_noise_;
  this->process_noise_matrix_ = Eigen::MatrixXd::Zero(3, 3);
  this->process_noise_matrix_(0, 0) = params.velocity_x_noise_;
  this->process_noise_matrix_(1, 1) = params.velocity_y_noise_;
  this->process_noise_matrix_(2, 2) = params.angular_velocity_noise_;
  this->observation_model_ = std::make_shared<ObservationModel>();
}

Eigen::MatrixXd EKFSLAMSolver::get_observation_noise_matrix(int num_landmarks) const {
  Eigen::MatrixXd observation_noise_matrix =
      Eigen::MatrixXd::Zero(num_landmarks * 2, num_landmarks * 2);
  for (int i = 0; i < num_landmarks; i++) {
    observation_noise_matrix(2 * i, 2 * i) = this->slam_parameters_.observation_x_noise_;
    observation_noise_matrix(2 * i + 1, 2 * i + 1) = this->slam_parameters_.observation_y_noise_;
  }
  return observation_noise_matrix;
}

void EKFSLAMSolver::add_velocities(const common_lib::structures::Velocities& velocities) {
  if (velocities_received_) {
    predict(this->state_, this->covariance_, process_noise_matrix_, this->last_update_, velocities);
  } else {
    velocities_received_ = true;
  }
  this->last_update_ = velocities.timestamp_;
}

void EKFSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
  int num_observations = static_cast<int>(cones.size());
  std::vector<int> matched_landmarks_indices;
  Eigen::VectorXd matched_observations(0);
  Eigen::VectorXd new_landmarks(0);
  Eigen::VectorXd new_confidences(0);
  Eigen::VectorXd observations(2 * num_observations);
  Eigen::VectorXd observation_confidences(num_observations);
  common_lib::conversions::cone_vector_to_eigen(cones, observations, observation_confidences);
  Eigen::VectorXd global_observations =
      common_lib::maths::local_to_global_coordinates(this->state_.segment(0, 3), observations);
  Eigen::VectorXi associations = this->_data_association_->associate(
      this->state_.segment(3, this->state_.size() - 3), global_observations,
      this->covariance_.block(3, 3, this->state_.size() - 3, this->state_.size() - 3),
      observation_confidences);
  for (int i = 0; i < num_observations; ++i) {
    if (associations(i) == -2) {
      continue;
    } else if (associations(i) == -1) {
      new_landmarks.conservativeResize(new_landmarks.size() + 2);
      new_landmarks(new_landmarks.size() - 2) = global_observations(2 * i);
      new_landmarks(new_landmarks.size() - 1) = global_observations(2 * i + 1);
      new_confidences.conservativeResize(new_confidences.size() + 1);
      new_confidences(new_confidences.size() - 1) = observation_confidences(i);
    } else {
      matched_landmarks_indices.push_back(associations(i) + 3);
      matched_observations.conservativeResize(matched_observations.size() + 2);
      matched_observations(matched_observations.size() - 2) = observations(2 * i);
      matched_observations(matched_observations.size() - 1) = observations(2 * i + 1);
    }
  }
  Eigen::VectorXd filtered_new_landmarks =
      this->_landmark_filter_->filter(new_landmarks, new_confidences);
  this->_landmark_filter_->delete_landmarks(filtered_new_landmarks);
  this->correct(this->state_, this->covariance_, matched_landmarks_indices, matched_observations);
  if (this->_mission_ != common_lib::competition_logic::Mission::NONE &&
      this->_mission_ != common_lib::competition_logic::Mission::SKIDPAD &&
      this->_mission_ != common_lib::competition_logic::Mission::ACCELERATION &&
      this->lap_counter_ == 0) {
    this->state_augmentation(
        this->state_, this->covariance_,
        filtered_new_landmarks);  // TODO: Erro aqui, isto são coordenadas relativas
    this->update_process_noise_matrix();
  }
}

void EKFSLAMSolver::predict(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
                            const Eigen::MatrixXd& process_noise_matrix,
                            const rclcpp::Time last_update,
                            const common_lib::structures::Velocities& velocities) {
  auto time_interval = velocities.timestamp_.seconds() - last_update.seconds();

  Eigen::Vector3d previous_pose = state.segment(0, 3);
  Eigen::Vector3d temp_velocities(velocities.velocity_x, velocities.velocity_y,
                                  velocities.rotational_velocity);

  Eigen::MatrixXd pose_jacobian =
      this->_motion_model_->get_jacobian_pose(previous_pose, temp_velocities, time_interval);
  int desired_jacobian_size = covariance.cols();
  Eigen::MatrixXd jacobian =
      Eigen::MatrixXd::Identity(desired_jacobian_size, desired_jacobian_size);
  jacobian.block(0, 0, 3, 3) = pose_jacobian;

  Eigen::Vector3d next_pose =
      this->_motion_model_->get_next_pose(previous_pose, temp_velocities, time_interval);
  state.segment(0, 3) = next_pose;
  this->pose = next_pose;
  covariance = jacobian * covariance * jacobian.transpose() + process_noise_matrix;
}

void EKFSLAMSolver::correct(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
                            const std::vector<int>& matched_landmarks_indices,
                            const Eigen::VectorXd& matched_observations) {
  Eigen::VectorXd predicted_observations =
      this->observation_model_->observation_model(state, matched_landmarks_indices);
  Eigen::MatrixXd jacobian =
      this->observation_model_->observation_model_jacobian(state, matched_landmarks_indices);
  Eigen::MatrixXd observation_noise_matrix =
      get_observation_noise_matrix(matched_landmarks_indices.size());
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + observation_noise_matrix).inverse();
  state += kalman_gain * (matched_observations - predicted_observations);
  covariance =
      (Eigen::MatrixXd::Identity(state.size(), state.size()) - kalman_gain * jacobian) * covariance;
}

void EKFSLAMSolver::state_augmentation(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
                                       const Eigen::VectorXd& new_landmarks_coordinates) {
  // Resize covariance matrix
  int num_new_entries = static_cast<int>(new_landmarks_coordinates.size());
  int covariance_size = static_cast<int>(covariance.rows());

  covariance.conservativeResizeLike(
      Eigen::MatrixXd::Zero(covariance_size + num_new_entries, covariance_size + num_new_entries));
  for (int i = 0; i < num_new_entries / 2; i++) {
    covariance(2 * i + covariance_size, 2 * i + covariance_size) =
        this->_params_.observation_x_noise_;
    covariance(2 * i + 1 + covariance_size, 2 * i + 1 + covariance_size) =
        _params_.observation_y_noise_;
  }
  // Resize state vector
  int original_state_size = static_cast<int>(state.size());
  state.conservativeResizeLike(Eigen::VectorXd::Zero(original_state_size + num_new_entries));
  state.segment(original_state_size, num_new_entries) = new_landmarks_coordinates;
}

void EKFSLAMSolver::load_initial_state(const Eigen::VectorXd& map, const Eigen::Vector3d& pose) {
  if (map.size() % 2 != 0 || pose.size() != 3) {
    throw std::runtime_error("Invalid map or pose size");
  }
  this->state_ = Eigen::VectorXd::Zero(3 + map.size());
  this->state_.segment(0, 3) = pose;
  this->state_.segment(3, map.size()) = map;
  this->covariance_ = Eigen::MatrixXd::Identity(this->state_.size(), this->state_.size()) *
                      this->_params_.preloaded_map_noise_;
  this->covariance_.block(0, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
  this->covariance_(0, 0) = this->slam_parameters_.pose_x_initial_noise_;
  this->covariance_(1, 1) = this->slam_parameters_.pose_y_initial_noise_;
  this->covariance_(2, 2) = this->slam_parameters_.pose_theta_initial_noise_;
  update_process_noise_matrix();
}

std::vector<common_lib::structures::Cone> EKFSLAMSolver::get_map_estimate() {
  std::vector<common_lib::structures::Cone> map;
  for (int i = 3; i < this->state_.size(); i += 2) {
    map.push_back(common_lib::structures::Cone(this->state_(i), this->state_(i + 1), "unknown", 1.0,
                                               this->last_update_));
  }
  return map;
}

common_lib::structures::Pose EKFSLAMSolver::get_pose_estimate() {
  return common_lib::structures::Pose(this->state_(0), this->state_(1), this->state_(2), 0, 0, 0,
                                      this->last_update_);
}

void EKFSLAMSolver::update_process_noise_matrix() {
  this->process_noise_matrix_ =
      Eigen::MatrixXd::Zero(this->covariance_.cols(), this->covariance_.rows());
  this->process_noise_matrix_(0, 0) = this->slam_parameters_.velocity_x_noise_;
  this->process_noise_matrix_(1, 1) = this->slam_parameters_.velocity_y_noise_;
  this->process_noise_matrix_(2, 2) = this->slam_parameters_.angular_velocity_noise_;
}