#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <iostream>

#include "loc_map/data_structures.hpp"
#include "utils/formulas.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(VehicleState* vehicle_state, Map* map,
                                           ImuUpdate* imu_update, Map* map_from_perception,
                                           Eigen::MatrixXf R, Eigen::MatrixXf Q,
                                           MotionModel& motion_model)
    : X(Eigen::VectorXf::Zero(200)),
      P(Eigen::MatrixXf::Zero(200, 200)),
      _vehicle_state(vehicle_state),
      _map(map),
      _imu_update(imu_update),
      _map_from_perception(map_from_perception),
      R(R),
      Q(Q),
      _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model) {}

void ExtendedKalmanFilter::prediction_step() {
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_last_update).count();
  MotionPredictionData prediction_data = {
      this->_imu_update->translational_velocity, this->_imu_update->translational_velocity_x,
      this->_imu_update->translational_velocity_y, this->_imu_update->rotational_velocity};
  X = this->_motion_model.motion_model_expected_state(X, prediction_data, delta / 1000000);
  P = this->_motion_model.motion_model_covariance_matrix(P, this->R, prediction_data,
                                                         delta / 1000000);
  // matrix in simulation
  this->_last_update = now;
}

void ExtendedKalmanFilter::correction_step() {
  // TODO(marhcouto): implement this
  // for (auto cone : this->_map_from_perception->map) {
  //   double absolute_x = cone.first.x + this->_vehicle_state->pose.position.x;
  //   double absolute_y = cone.first.y + this->_vehicle_state->pose.position.y;

  // }
}

void ExtendedKalmanFilter::update() { this->_vehicle_state->pose = Pose(X(0), X(1), X(2)); }

// ------------------ Motion Models ------------------

// Not working
Eigen::VectorXf NormalVelocityModel::motion_model_expected_state(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) {
  Eigen::VectorXf next_state = expected_state;
  if (motion_prediction_data.rotational_velocity == 0.0) {  // Rectilinear movement
    next_state(0) +=
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
    next_state(1) +=
        motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    next_state(0) +=
        -(motion_prediction_data.translational_velocity /
          motion_prediction_data.rotational_velocity) *
            sin(expected_state(2)) +
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            sin(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
    next_state(1) +=
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            cos(expected_state(2)) -
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            cos(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  }
  next_state(2) = normalize_angle(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);
  return next_state;
}

Eigen::MatrixXf NormalVelocityModel::motion_model_covariance_matrix(
    const Eigen::MatrixXf& state_covariance_matrix, const Eigen::MatrixXf& motion_noise_matrix,
    const MotionPredictionData& motion_prediction_data, const double time_interval) {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(state_covariance_matrix.rows(), state_covariance_matrix.cols());

  if (motion_prediction_data.rotational_velocity == 0.0) {  // Rectilinear movement
    jacobian(0, 2) = -motion_prediction_data.translational_velocity *
                     sin(state_covariance_matrix(2)) * time_interval;
    jacobian(1, 2) = motion_prediction_data.translational_velocity *
                     cos(state_covariance_matrix(2)) * time_interval;
  } else {  // Curvilinear movement
    jacobian(0, 2) = -(motion_prediction_data.translational_velocity /
                       motion_prediction_data.rotational_velocity) *
                         cos(state_covariance_matrix(2)) +
                     (motion_prediction_data.translational_velocity /
                      motion_prediction_data.rotational_velocity) *
                         cos(state_covariance_matrix(2) +
                             motion_prediction_data.rotational_velocity * time_interval);
    jacobian(1, 2) = -(motion_prediction_data.translational_velocity /
                       motion_prediction_data.rotational_velocity) *
                         sin(state_covariance_matrix(2)) +
                     (motion_prediction_data.translational_velocity /
                      motion_prediction_data.rotational_velocity) *
                         sin(state_covariance_matrix(2) +
                             motion_prediction_data.rotational_velocity * time_interval);
  }
  Eigen::MatrixXf new_state_covariance_matrix =
      jacobian * state_covariance_matrix * jacobian.transpose() + motion_noise_matrix;

  return new_state_covariance_matrix;
}

Eigen::VectorXf ImuVelocityModel::motion_model_expected_state(
    const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
    const double time_interval) {
  Eigen::VectorXf next_state = expected_state;
  next_state(0) += motion_prediction_data.translational_velocity_x * time_interval;
  next_state(1) += motion_prediction_data.translational_velocity_y * time_interval;
  next_state(2) =
      normalize_angle(next_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  return next_state;
}

// TODO(marhcouto): check what to do about the unused paramter warnings
Eigen::MatrixXf ImuVelocityModel::motion_model_covariance_matrix(
    const Eigen::MatrixXf& state_covariance_matrix, const Eigen::MatrixXf& motion_noise_matrix,
    const MotionPredictionData& motion_prediction_data,
    const double time_interval) {  // In this implementation, as the motion model is already linear,
                                   // we do not use the derivative of the model
  Eigen::MatrixXf motion_to_state_matrix =
      Eigen::MatrixXf::Identity(state_covariance_matrix.rows(), state_covariance_matrix.cols());
  Eigen::MatrixXf new_state_covariance_matrix =
      motion_to_state_matrix * state_covariance_matrix * motion_to_state_matrix.transpose() +
      motion_noise_matrix;

  return new_state_covariance_matrix;
}