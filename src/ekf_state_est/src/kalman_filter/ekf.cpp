#define EIGEN_USE_THREADS

#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/matrixes.hpp"

/*---------------------- Tests helper functions --------------------*/

void ExtendedKalmanFilter::set_X_y(int y, float value) { this->x_vector_(y) = value; }

void ExtendedKalmanFilter::set_P(int size) {
  this->p_matrix_ = Eigen::SparseMatrix<float>(size, size);
  this->p_matrix_.coeffRef(0, 0) = 1.1;
  this->p_matrix_.coeffRef(1, 1) = 1.1;
  this->p_matrix_.coeffRef(2, 2) = 1.1;
}

/**
 * @brief Initializes state x_vector_ with size equal to parameter size
 */
void ExtendedKalmanFilter::init_X_size(int size) { this->x_vector_ = Eigen::VectorXf::Zero(size); }

/*---------------------- Constructors --------------------*/

ExtendedKalmanFilter::ExtendedKalmanFilter(const MotionModel &motion_model,
                                           const ObservationModel &observation_model,
                                           const DataAssociationModel &data_association_model)
    : p_matrix_(5, 5),
      _motion_model_(motion_model),
      _observation_model_(observation_model),
      _data_association_model_(data_association_model) {
  p_matrix_.setZero();
}
/*-----------------------Algorithms-----------------------*/

void ExtendedKalmanFilter::prediction_step(const MotionUpdate &motion_update) {
  double delta = (motion_update.last_update - this->_last_update_).seconds();
  this->_last_update_ = motion_update.last_update;
  if (this->_first_prediction_) {  // To set the last update correctly
    this->_first_prediction_ = false;
    return;
  }
  Eigen::VectorXf temp_x_vector = this->x_vector_;
  this->x_vector_ =
      this->_motion_model_.predict_expected_state(temp_x_vector, motion_update, delta / 1000000);
  Eigen::MatrixXf g_matrix = this->_motion_model_.get_motion_to_state_matrix(
      temp_x_vector, motion_update, delta / 1000000);
  Eigen::MatrixXf r_matrix = this->_motion_model_.get_process_noise_covariance_matrix(
      static_cast<unsigned int>(this->x_vector_.size()));  // Process Noise Matrix
  // this->p_matrix_ = g_matrix * this->p_matrix_ * g_matrix.transpose() + r_matrix;
  Eigen::MatrixXf p_matrix_dense =
      g_matrix * Eigen::MatrixXf(this->p_matrix_) * g_matrix.transpose() + r_matrix;
  this->p_matrix_ = p_matrix_dense.sparseView();
}

void ExtendedKalmanFilter::correction_step(
    const std::vector<common_lib::structures::Cone> &perception_map) {
  for (auto cone : perception_map) {
    ObservationData observation_data =
        ObservationData(cone.position.x, cone.position.y, cone.color);

    // Data Association
    Eigen::Vector2f landmark_absolute =
        this->_observation_model_.inverse_observation_model(this->x_vector_, observation_data);
    int landmark_index = _data_association_model_.match_cone(landmark_absolute, this->x_vector_);
    if (landmark_index == -1) {  // Too far away landmark
      continue;
    } else if (landmark_index == -2) {  // Did not match any landmark
      this->x_vector_.conservativeResizeLike(Eigen::VectorXf::Zero(this->x_vector_.size() + 2));
      this->x_vector_(this->x_vector_.size() - 2) = landmark_absolute(0);
      this->x_vector_(this->x_vector_.size() - 1) = landmark_absolute(1);
      Eigen::SparseMatrix<float> new_p_matrix(this->p_matrix_.rows() + 2,
                                              this->p_matrix_.cols() + 2);
      common_lib::maths::copy_eigen_sparse_matrix(this->p_matrix_, new_p_matrix);
      this->p_matrix_.swap(new_p_matrix);
      landmark_index = static_cast<int>(this->x_vector_.size() - 2);
    }

    // Matrix Calculation
    Eigen::MatrixXf h_matrix = this->_observation_model_.get_state_to_observation_matrix(
        this->x_vector_, landmark_index, static_cast<unsigned int>(this->x_vector_.size()));
    Eigen::MatrixXf q_matrix =
        this->_observation_model_.get_observation_noise_covariance_matrix();  // Observation Noise
                                                                              // Matrix
    Eigen::MatrixXf kalman_gain_matrix = this->get_kalman_gain(h_matrix, this->p_matrix_, q_matrix);

    // Correction
    Eigen::Vector2f z_hat = this->_observation_model_.observation_model(
        this->x_vector_, landmark_index);  // expected observation, previously
                                           // calculated landmark is used
    Eigen::Vector2f z =
        Eigen::Vector2f(observation_data.position.x,
                        observation_data.position.y);  // position of the landmark  observed
    this->x_vector_ = this->x_vector_ + kalman_gain_matrix * (z - z_hat);
    // TODO(PedroRomao3): divide into multiple calculation and measure time.
    // TODO(PedroRomao3): Diagram.
    this->p_matrix_ = (Eigen::MatrixXf::Identity(this->p_matrix_.rows(), this->p_matrix_.cols()) -
                       kalman_gain_matrix * h_matrix)
                          .sparseView() *
                      this->p_matrix_;
  }
}

Eigen::MatrixXf ExtendedKalmanFilter::get_kalman_gain(const Eigen::MatrixXf &h_matrix,
                                                      const Eigen::MatrixXf &p_matrix,
                                                      const Eigen::MatrixXf &q_matrix) {
  Eigen::MatrixXf s_matrix = h_matrix * p_matrix * h_matrix.transpose() + q_matrix;
  Eigen::MatrixXf kalman_gain_matrix = p_matrix * h_matrix.transpose() * s_matrix.inverse();
  return kalman_gain_matrix;
}

void ExtendedKalmanFilter::update(
    std::shared_ptr<common_lib::structures::VehicleState> vehicle_state,
    std::shared_ptr<std::vector<common_lib::structures::Cone>> track_map) {
  vehicle_state->pose.position.x = this->x_vector_(0);
  vehicle_state->pose.position.y = this->x_vector_(1);
  vehicle_state->pose.orientation = this->x_vector_(2);
  vehicle_state->linear_velocity = this->x_vector_(3);
  vehicle_state->angular_velocity = this->x_vector_(4);
  track_map->clear();
  for (int i = 5; i < this->x_vector_.size() - 1; i += 2) {
    track_map->push_back(
        common_lib::structures::Cone(x_vector_(i), x_vector_(i + 1), "UNKNOWN", 1.0));
  }
}
