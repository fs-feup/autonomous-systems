#include <cmath>
#include <Eigen/Dense>

#include "kalman_filter/ekf.hpp"
#include "utils/position.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                           const Eigen::MatrixXd& C, const Eigen::MatrixXd& R,
                                           const Eigen::MatrixXd& Q)
    : A(A),
      B(B),
      C(C),
      R(R),
      Q(Q),
      expected_state(Eigen::ArrayXf::Zero(200)),
      P(Eigen::MatrixXf::Zero(200)){};

void ExtendedKalmanFilter::next_state(Eigen::VectorXd& control, Eigen::VectorXd& measurement) {}


Eigen::VectorXd motion_model_expected_state(const Eigen::VectorXd& expected_state, const float translational_velocity, 
const float rotational_velocity, const double time_interval) {
  Eigen::Vector3d next_state;
  next_state(0) = expected_state(0) - (translational_velocity / rotational_velocity) * sin(expected_state(2)) + (translational_velocity / rotational_velocity) * sin(expected_state(2) + rotational_velocity * time_interval);
  next_state(1) = expected_state(1) + (translational_velocity / rotational_velocity) * cos(expected_state(2)) - (translational_velocity / rotational_velocity) * cos(expected_state(2) + rotational_velocity * time_interval);
  next_state(2) = expected_state(2) + rotational_velocity * time_interval;
  return next_state;
}

Eigen::MatrixXd motion_model_covariance_matrix(const Eigen::MatrixXd& state_covariance_matrix, const Eigen::MatrixXd& motion_noise_covariance_matrix, const float translational_velocity,
const float rotational_velocity, const double time_interval) {
  Eigen::MatrixXd new_state_covariance_matrix = state_covariance_matrix;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(state_covariance_matrix.rows(), state_covariance_matrix.cols());
  jacobian(0, 2) = - (translational_velocity / rotational_velocity) * cos(state_covariance_matrix(2)) + (translational_velocity / rotational_velocity) * cos(state_covariance_matrix(2) + rotational_velocity * time_interval);
  jacobian(1, 2) = - (translational_velocity / rotational_velocity) * sin(state_covariance_matrix(2)) + (translational_velocity / rotational_velocity) * sin(state_covariance_matrix(2) + rotational_velocity * time_interval);
  new_state_covariance_matrix = jacobian * state_covariance_matrix * jacobian.transpose() * motion_noise_covariance_matrix;
  
  return new_state_covariance_matrix;
}