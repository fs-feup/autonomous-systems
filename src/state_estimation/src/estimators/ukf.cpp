#include "estimators/ukf.hpp"

UKF::UKF(SEParameters se_parameters) : params_(se_parameters), last_update_(rclcpp::Clock().now()) {
  // Initialize the process noise matrix, wheels measurement noise matrix, and IMU measurement noise
  // matrix
  process_noise_matrix_ = Eigen::Matrix<double, 10, 10>::Zero();
  wheels_measurement_noise_matrix_ = Eigen::Matrix<double, 10, 10>::Zero();
  imu_measurement_noise_matrix_ = Eigen::Matrix<double, 10, 10>::Zero();
  lambda_ = se_parameters.alpha * se_parameters.alpha * (10 + se_parameters.kappa) - 10;

  // Compute sigma point weights
  int n = state_.size();
  weights_ = Eigen::VectorXd(2 * n + 1);
  weights_(0) = lambda_ / (n + lambda_);
  for (int i = 1; i < 2 * n + 1; i++) {
    weights_(i) = 1 / (2 * (n + lambda_));
  }

  // TODO: set the process noise matrix, wheels measurement noise matrix, and IMU measurement noise
  // etc..
}

void UKF::compute_sigma_points(const State& state, const Eigen::Matrix<double, 10, 10>& covariance,
                               Eigen::Matrix<State, -1, 1>& sigma_points) {
  // Use the Merwe Scaled Sigma Points method to compute the sigma points.
  // The number of sigma points is 2n + 1, where n is the dimension of the state vector.
  int n = state.size();
  int num_sigma_points = 2 * n + 1;
  sigma_points.resize(num_sigma_points);

  // Compute the square root of the covariance matrix using Cholesky decomposition.
  Eigen::MatrixXd scaled_covariance = (n + lambda_) * covariance;
  Eigen::LLT<Eigen::MatrixXd> llt(scaled_covariance);
  Eigen::MatrixXd sqrt_covariance = llt.matrixL();

  // Compute the actual sigma points.
  sigma_points(0) = state;
  for (int i = 0; i < n; i++) {
    sigma_points(i + 1) = state + sqrt_covariance.col(i);
    sigma_points(i + n + 1) = state - sqrt_covariance.col(i);
  }

  return;
}