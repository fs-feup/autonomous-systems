#include "slam_solver/ekf_slam_solver.hpp"

Eigen::MatrixXd EKFSLAMSolver::get_observation_noise_matrix(int num_landmarks) const {
  Eigen::MatrixXd observation_noise_matrix =
      Eigen::MatrixXd::Zero(num_landmarks * 2, num_landmarks * 2);
  for (int i = 0; i < num_landmarks; i++) {
    observation_noise_matrix(2 * i, 2 * i) = 0;  // this->slam_parameters_.observation_x_noise_;
    observation_noise_matrix(2 * i + 1, 2 * i + 1) =
        0;  // this->slam_parameters_.observation_y_noise_;
  }
  return observation_noise_matrix;
}

void EKFSLAMSolver::add_motion_prior(const common_lib::structures::Velocities& velocities) {
  if (velocities_received_ && cones_receieved_) {
    predict(this->state_, this->covariance_, process_noise_matrix_, this->last_update_, velocities);
  } else {
    velocities_received_ = true;
  }
  this->last_update_ = velocities.timestamp;
}

void EKFSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
  Eigen::VectorXd landmarks = Eigen::VectorXd::Zero(2 * cones.size());
  for (int i = 0; i < static_cast<int>(cones.size()); ++i) {
    landmarks(2 * i) = cones[i].position.x;
    landmarks(2 * i + 1) = cones[i].position.y;
  }
  std::vector<int> matched_landmarks_indices;
  Eigen::VectorXd matched_observations;
  Eigen::VectorXd new_landmarks;
  // TODO: call data_association(this->state, this->observed_landmarks)
  correct(this->state_, this->covariance_, matched_landmarks_indices, matched_observations);
  state_augmentation(this->state_, this->covariance_, new_landmarks);
}

void EKFSLAMSolver::predict(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
                            const Eigen::MatrixXd& process_noise_matrix,
                            const rclcpp::Time last_update,
                            const common_lib::structures::Velocities& velocities) {
  auto time_interval = velocities.timestamp.seconds() - last_update.seconds();

  Eigen::Vector3d previous_pose = state.segment(0, 3);
  Eigen::Vector3d temp_velocities(velocities.velocity_x, velocities.velocity_y,
                                  velocities.rotational_velocity);

  Eigen::MatrixXd jacobian =
      this->_motion_model_->get_jacobian(previous_pose, temp_velocities, time_interval);

  Eigen::Vector3d next_pose =
      this->_motion_model_->get_next_pose(previous_pose, temp_velocities, time_interval);
  state.segment(0, 3) = next_pose;
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
                                       const Eigen::VectorXd& new_landmarks) {
  // Resize covariance matrix
  int num_landmarks = static_cast<int>(new_landmarks.size()) / 2;
  int covariance_rows = static_cast<int>(covariance.rows());
  int covariance_cols = static_cast<int>(covariance.cols());
  Eigen::MatrixXd Pcc = covariance.block(0, 0, 3, 3);
  Eigen::MatrixXd Plc = covariance.block(3, 0, covariance_rows - 3, 3);

  Eigen::MatrixXd gv =
      this->observation_model_->inverse_observation_model_jacobian_pose(state, new_landmarks);
  Eigen::MatrixXd GvT = gv.transpose();
  Eigen::MatrixXd gz =
      this->observation_model_->inverse_observation_model_jacobian_landmarks(state, new_landmarks);

  Eigen::MatrixXd R = this->get_observation_noise_matrix(num_landmarks);

  Eigen::MatrixXd Pcn = Pcc * GvT;
  Eigen::MatrixXd Pln = Plc * GvT;
  Eigen::MatrixXd Pnn = gv * Pcc * GvT + gz * R * gz.transpose();

  covariance.conservativeResizeLike(Eigen::MatrixXd::Zero(covariance_rows + 2 * num_landmarks,
                                                          covariance_cols + 2 * num_landmarks));

  covariance.block(0, covariance_cols, 3, 2 * num_landmarks) = Pcn;
  covariance.block(3, covariance_cols, covariance_rows - 3, 2 * num_landmarks) = Pln;
  covariance.block(covariance_rows, covariance_cols, 2 * num_landmarks, 2 * num_landmarks) = Pnn;
  covariance.block(covariance_rows, 0, 2 * num_landmarks, 3) = Pcn.transpose();
  covariance.block(covariance_cols, 3, 2 * num_landmarks, covariance_cols - 3) = Pln.transpose();

  // Resize state vector
  int original_state_size = static_cast<int>(state.size());
  state.conservativeResizeLike(Eigen::VectorXd::Zero(original_state_size + 2 * num_landmarks));
  state.segment(original_state_size, 2 * num_landmarks) =
      this->observation_model_->inverse_observation_model(state, new_landmarks);
}
