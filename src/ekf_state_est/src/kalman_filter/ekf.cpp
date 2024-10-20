#define EIGEN_USE_THREADS

#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/matrixes.hpp"

/*---------------------- Constructors --------------------*/

ExtendedKalmanFilter::ExtendedKalmanFilter(
    std::shared_ptr<ObservationModel> observation_model,
    std::shared_ptr<DataAssociationModel> data_association_model)
    : _observation_model_(observation_model), _data_association_model_(data_association_model) {
  //_p_matrix_ = Eigen::MatrixXf::Identity(6, 6) * 100.0f;
  _x_vector_ = Eigen::VectorXf::Constant(6, (float)0.0000001);
}
/*-----------------------Algorithms-----------------------*/

void ExtendedKalmanFilter::prediction_step(const MotionUpdate &motion_update,
                                           const std::string &sensor_type) {
  std::shared_ptr<MotionModel> motion_model = _motion_models_[sensor_type];
  rclcpp::Time temp_this_last_update = this->_last_update_;
  this->_last_update_ = motion_update.last_update;
  if (this->_first_prediction_) {  // To set the last update correctly
    this->_first_prediction_ = false;
    return;
  }

  double delta = (this->_last_update_ - temp_this_last_update).seconds();
  Eigen::VectorXf temp_x_vector = this->_x_vector_;
  this->_x_vector_ = motion_model->predict_expected_state(temp_x_vector, motion_update, delta);

  Eigen::MatrixXf g_matrix =
      motion_model->get_motion_to_state_matrix(temp_x_vector, motion_update, delta);
  Eigen::MatrixXf r_matrix = motion_model->get_process_noise_covariance_matrix(
      static_cast<unsigned int>(this->_x_vector_.size()));  // Process Noise Matrix

  this->_p_matrix_ = g_matrix * Eigen::MatrixXf(this->_p_matrix_) * g_matrix.transpose() + r_matrix;
}

std::string track_to_string(Eigen::VectorXf state) {
  std::string track = "";
  for (int i = 6; i < state.size(); i += 2) {
    track += "(" + std::to_string(state(i)) + "," + std::to_string(state(i + 1)) + "), ";
  }
  return track;
}

void ExtendedKalmanFilter::correction_step(
    const std::vector<common_lib::structures::Cone> &perception_map) {
  // loop through all the cones in the perception map
  // init vector for matched ids and cone positions
  std::vector<int> matched_ids;
  std::vector<Eigen::Vector2f> matched_cone_positions;
  // init vector for new features with new cone positions
  std::vector<Eigen::Vector2f> new_features;
  this->_data_association_model_->associate_n_filter(
      perception_map, this->_x_vector_, this->_p_matrix_, matched_ids, matched_cone_positions,
      new_features, this->_observation_model_.get());
  // loop through the matched ids and cone positions and make the corrections

  correct_with_matched_ids_full_state(matched_ids, matched_cone_positions);

  // loop through the new features and add them to the state vector
  augment_state(new_features);
}

// loop through the matched ids and cone positions and make the corrections
void ExtendedKalmanFilter::correct_with_matched_ids(
    const std::vector<int> &matched_ids,
    const std::vector<Eigen::Vector2f> &matched_cone_positions) {
  for (int i = 0; i < matched_ids.size(); i++) {
    Eigen::Vector2f z_hat =
        this->_observation_model_->observation_model(this->_x_vector_, matched_ids[i]);
    Eigen::MatrixXf h_matrix = this->_observation_model_->get_state_to_observation_matrix(
        this->_x_vector_, matched_ids[i], static_cast<unsigned int>(this->_x_vector_.size()));
    Eigen::MatrixXf q_matrix = this->_observation_model_->get_observation_noise_covariance_matrix();
    Eigen::Vector2f z = matched_cone_positions[i];

    Eigen::MatrixXf kalman_gain_matrix =
        this->get_kalman_gain(h_matrix, this->_p_matrix_, q_matrix);
    this->_x_vector_ = this->_x_vector_ + kalman_gain_matrix * (z - z_hat);
    this->_p_matrix_ =
        (Eigen::MatrixXf::Identity(this->_p_matrix_.rows(), this->_p_matrix_.cols()) -
         kalman_gain_matrix * h_matrix)
            .sparseView() *  // for some reason, this is needed otherwise the map created is absurd
        this->_p_matrix_;
  }
}

void ExtendedKalmanFilter::correct_with_matched_ids_full_state(
    const std::vector<int> &matched_ids,
    const std::vector<Eigen::Vector2f> &matched_cone_positions) {
  Eigen::VectorXf observations =
      this->_observation_model_->format_observation(matched_cone_positions);
  Eigen::VectorXf expected_observations =
      this->_observation_model_->observation_model_n_landmarks(this->_x_vector_, matched_ids);
  Eigen::MatrixXf Q = this->_observation_model_->get_full_observation_noise_covariance_matrix(
      static_cast<int>(observations.size()));
  Eigen::MatrixXf H =
      this->_observation_model_->get_jacobian_of_observation_model(this->_x_vector_, matched_ids);
  Eigen::MatrixXf P = this->_p_matrix_;
  Eigen::MatrixXf S = H * P * H.transpose() + Q;
  Eigen::MatrixXf K = P * H.transpose() * S.inverse();
  this->_x_vector_ += K * (observations - expected_observations);
  this->_p_matrix_ = (Eigen::MatrixXf::Identity(P.rows(), P.cols()) - K * H) * P;
}

// loop through the new features and add them to the state vector
void ExtendedKalmanFilter::augment_state(const std::vector<Eigen::Vector2f> &new_features) {
  for (int i = 0; i < new_features.size(); i++) {
    int length = this->_x_vector_.size();
    this->_x_vector_.conservativeResizeLike(Eigen::VectorXf::Zero(this->_x_vector_.size() + 2));
    // call inverse observational model ot change the cone position to the car frame
    Eigen::Vector2f landmark_absolute = this->_observation_model_->inverse_observation_model(
        this->_x_vector_, ObservationData(new_features[i](0), new_features[i](1),
                                          common_lib::competition_logic::Color::BLUE));
    this->_x_vector_(this->_x_vector_.size() - 2) = landmark_absolute(0);
    this->_x_vector_(this->_x_vector_.size() - 1) = landmark_absolute(1);

    Eigen::MatrixXf new_p_matrix =
        Eigen::MatrixXf::Zero(this->_p_matrix_.rows() + 2, this->_p_matrix_.cols() + 2);
    new_p_matrix.block(0, 0, this->_p_matrix_.rows(), this->_p_matrix_.cols()) = this->_p_matrix_;
    this->_p_matrix_ = new_p_matrix;

    Eigen::MatrixXf Gv = _observation_model_->get_gv(
        _x_vector_, ObservationData(new_features[i](0), new_features[i](1),
                                    common_lib::competition_logic::Color::BLUE));

    Eigen::MatrixXf Gz = _observation_model_->get_gz(
        _x_vector_, ObservationData(new_features[i](0), new_features[i](1),
                                    common_lib::competition_logic::Color::BLUE));
    Eigen::Matrix2f R = _observation_model_->get_observation_noise_covariance_matrix();

    Eigen::MatrixXf GvT = Gv.transpose();
    Eigen::MatrixXf GzT = Gz.transpose();

    Eigen::MatrixXf P_submatrix = this->_p_matrix_.block(0, 0, 3, 3);

    Eigen::MatrixXf new_submatrix = Gv * P_submatrix * GvT + Gz * R * GzT;

    this->_p_matrix_.block(length, length, 2, 2) = new_submatrix;

    P_submatrix = this->_p_matrix_.block(0, 0, 3, 3);
    new_submatrix = Gv * P_submatrix;
    this->_p_matrix_.block(length, 0, 2, 3) = new_submatrix;

    P_submatrix = this->_p_matrix_.block(length, 0, 2, 3);
    new_submatrix = P_submatrix.transpose();
    this->_p_matrix_.block(0, length, 3, 2) = new_submatrix;

    if (length > 6) {
      this->_p_matrix_.block(length, 6, 2, length - 6) =
          Gv * this->_p_matrix_.block(0, 6, 3, length - 6);
      this->_p_matrix_.block(6, length, length - 6, 2) =
          (this->_p_matrix_.block(length, 6, 2, length - 6)).transpose();
    }
  }
}

Eigen::MatrixXf ExtendedKalmanFilter::get_kalman_gain(const Eigen::MatrixXf &h_matrix,
                                                      const Eigen::MatrixXf &p_matrix,
                                                      const Eigen::MatrixXf &q_matrix) const {
  Eigen::MatrixXf s_matrix = h_matrix * p_matrix * h_matrix.transpose() + q_matrix;

  if ((s_matrix.array() == 0).all()) {
    return Eigen::MatrixXf::Zero(p_matrix.rows(), h_matrix.rows());
  }
  Eigen::MatrixXf kalman_gain_matrix = p_matrix * h_matrix.transpose() * s_matrix.inverse();

  return kalman_gain_matrix;
}

void ExtendedKalmanFilter::update(
    std::shared_ptr<common_lib::structures::VehicleState> vehicle_state,
    std::shared_ptr<std::vector<common_lib::structures::Cone>> track_map) {
  vehicle_state->pose.position.x = this->_x_vector_(0);
  vehicle_state->pose.position.y = this->_x_vector_(1);
  vehicle_state->pose.orientation = this->_x_vector_(2);
  vehicle_state->velocity_x = this->_x_vector_(3);
  vehicle_state->velocity_y = this->_x_vector_(4);
  vehicle_state->rotational_velocity = this->_x_vector_(5);
  track_map->clear();
  for (int i = 6; i < this->_x_vector_.size() - 1; i += 2) {
    track_map->push_back(
        common_lib::structures::Cone(_x_vector_(i), _x_vector_(i + 1), "UNKNOWN", 1.0));
  }
}
