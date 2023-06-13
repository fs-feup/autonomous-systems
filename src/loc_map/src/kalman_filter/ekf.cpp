#include <Eigen/Dense>
#include <iostream>

#include "kalman_filter/ekf.hpp"
#include "utils/formulas.hpp"
#include "loc_map/data_structures.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(VehicleState* vehicle_state, Map* map, ImuUpdate* imu_update, MotionModel& motion_model)
    : X(Eigen::VectorXf::Zero(200)),
      P(Eigen::MatrixXf::Zero(200, 200)),
      _vehicle_state(vehicle_state),
      _map(map),
      _imu_update(imu_update),
      _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model) {}


void ExtendedKalmanFilter::prediction_step() {
    std::chrono::time_point<std::chrono::high_resolution_clock> now =
        std::chrono::high_resolution_clock::now();
  double delta = std::chrono::duration_cast<std::chrono::microseconds>(
                     now - this->_last_update).count();
  MotionPredictionData prediction_data = {
      this->_imu_update->translational_velocity,
      this->_imu_update->translational_velocity_x,
      this->_imu_update->translational_velocity_y,
      this->_imu_update->rotational_velocity};
  X = this->_motion_model.motion_model_expected_state(X, prediction_data, delta / 1000000);
  P = this->_motion_model.motion_model_covariance_matrix(P, prediction_data, delta / 1000000); // TODO(marhcouto): introduce multiplication by the noise
                                       // matrix in simulation
  this->_last_update = now;
}

void ExtendedKalmanFilter::update() {
    this->_vehicle_state->pose = Pose(X(0), X(1), X(2));
}




// ------------------ Motion Models ------------------

// Not working
Eigen::VectorXf NormalVelocityModel::motion_model_expected_state(const Eigen::VectorXf& expected_state,
                                            MotionPredictionData motion_prediction_data,
                                            const double time_interval) {
  Eigen::VectorXf next_state = expected_state;
  if (motion_prediction_data.rotational_velocity == 0.0) { // Rectilinear movement
    next_state(0) = expected_state(0) - motion_prediction_data.translational_velocity * sin_in_degrees(expected_state(2)) * time_interval; 
    next_state(1) = expected_state(0) + motion_prediction_data.translational_velocity * cos_in_degrees(expected_state(2)) * time_interval;                 
  } else { // Curvilinear movement
    next_state(0) = expected_state(0) - (motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) * sin_in_degrees(expected_state(2)) +
                  (motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) *
                      sin_in_degrees(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);  
    next_state(1) = expected_state(1) + (motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) * cos_in_degrees(expected_state(2)) -
                  (motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) *
                      cos_in_degrees(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  }
  next_state(2) = expected_state(2) + motion_prediction_data.rotational_velocity * time_interval;
  return next_state;
}

// Not working
Eigen::MatrixXf NormalVelocityModel::motion_model_covariance_matrix(
    const Eigen::MatrixXf& state_covariance_matrix,
    MotionPredictionData motion_prediction_data, const double time_interval) {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(state_covariance_matrix.rows(), state_covariance_matrix.cols());
  jacobian(0, 2) =
      -(motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) * cos_in_degrees(state_covariance_matrix(2)) +
      (motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) *
          cos_in_degrees(state_covariance_matrix(2) + motion_prediction_data.rotational_velocity * time_interval); // TODO: check how this is made
  jacobian(1, 2) =
      -(motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) * sin_in_degrees(state_covariance_matrix(2)) +
      (motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity) *
          sin_in_degrees(state_covariance_matrix(2) + motion_prediction_data.rotational_velocity * time_interval);
  Eigen::MatrixXf new_state_covariance_matrix =
      jacobian * state_covariance_matrix *
      jacobian.transpose();  // TODO(marhcouto): introduce multiplication
                             // by the noise matrix in simulation

  return new_state_covariance_matrix;
}

Eigen::VectorXf ImuVelocityModel::motion_model_expected_state(
    const Eigen::VectorXf& expected_state, MotionPredictionData motion_prediction_data, const double time_interval) {
  Eigen::VectorXf next_state = expected_state;
  next_state(0) += motion_prediction_data.translational_velocity_x * time_interval;
  next_state(1) += motion_prediction_data.translational_velocity_y * time_interval;
  next_state(2) = normalize_angle(next_state(2) + motion_prediction_data.rotational_velocity * time_interval);
  return next_state;
}

Eigen::MatrixXf ImuVelocityModel::motion_model_covariance_matrix(
    const Eigen::MatrixXf& state_covariance_matrix,
    MotionPredictionData motion_prediction_data,
    const double time_interval) { // In this implementation, as the motion model is already linear, we do not use the derivative of the model
  Eigen::MatrixXf motion_to_state_matrix =
      Eigen::MatrixXf::Identity(state_covariance_matrix.rows(), state_covariance_matrix.cols());
  motion_to_state_matrix(0, 2) = motion_prediction_data.translational_velocity_x * time_interval; // 
  motion_to_state_matrix(1, 2) = motion_prediction_data.translational_velocity_y * time_interval;
  motion_to_state_matrix(2, 2) = motion_prediction_data.rotational_velocity * time_interval;
  Eigen::MatrixXf new_state_covariance_matrix =
      motion_to_state_matrix * state_covariance_matrix *
      motion_to_state_matrix.transpose();  // TODO: introduce multiplication by the noise matrix in simulation

  return new_state_covariance_matrix;
}