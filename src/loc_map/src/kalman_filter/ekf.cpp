#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <cmath>

#include "utils/position.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(VehicleState* state, Map* map, ImuUpdate* imu_update)
    : _vehicle_state(state),
      _map(map),
      _imu_update(imu_update),
      X(Eigen::ArrayXf::Zero(200)),
      P(Eigen::MatrixXf::Zero(200)){};

Eigen::VectorXd motion_model_expected_state(const Eigen::VectorXd& expected_state,
                                            const float translational_velocity,
                                            const float rotational_velocity,
                                            const double time_interval) {
  Eigen::Vector3d next_state = expected_state;
  next_state(0) = expected_state(0) -
                  (translational_velocity / rotational_velocity) * sin(expected_state(2)) +
                  (translational_velocity / rotational_velocity) *
                      sin(expected_state(2) + rotational_velocity * time_interval);
  next_state(1) = expected_state(1) +
                  (translational_velocity / rotational_velocity) * cos(expected_state(2)) -
                  (translational_velocity / rotational_velocity) *
                      cos(expected_state(2) + rotational_velocity * time_interval);
  next_state(2) = expected_state(2) + rotational_velocity * time_interval;
  return next_state;
}

Eigen::MatrixXd motion_model_covariance_matrix(
    const Eigen::MatrixXd& state_covariance_matrix,
    const Eigen::MatrixXd& motion_noise_covariance_matrix, const float translational_velocity,
    const float rotational_velocity, const double time_interval) {
  Eigen::MatrixXd new_state_covariance_matrix = state_covariance_matrix;
  Eigen::MatrixXd jacobian =
      Eigen::MatrixXd::Identity(state_covariance_matrix.rows(), state_covariance_matrix.cols());
  jacobian(0, 2) =
      -(translational_velocity / rotational_velocity) * cos(state_covariance_matrix(2)) +
      (translational_velocity / rotational_velocity) *
          cos(state_covariance_matrix(2) + rotational_velocity * time_interval);
  jacobian(1, 2) =
      -(translational_velocity / rotational_velocity) * sin(state_covariance_matrix(2)) +
      (translational_velocity / rotational_velocity) *
          sin(state_covariance_matrix(2) + rotational_velocity * time_interval);
  new_state_covariance_matrix = jacobian * state_covariance_matrix *
                                jacobian.transpose();  // TODO(marhcouto): introduce multiplication
                                                       // by the noise matrix in simulation

  return new_state_covariance_matrix;
}

void ExtendedKalmanFilter::prediction_step() {
  X = motion_model_expected_state(X, this->_imu_update->translational_velocity,
                                  this->_imu_update->rotational_velocity,
                                  this->_imu_update->delta_time);
  P = motion_model_covariance_matrix(
      P, Eigen::MatrixXd::Identity(P.rows(), P.cols()), this->_imu_update->translational_velocity,
      this->_imu_update->rotational_velocity,
      this->_imu_update->delta_time);  // TODO(marhcouto): introduce multiplication by the noise
                                       // matrix in simulation
}