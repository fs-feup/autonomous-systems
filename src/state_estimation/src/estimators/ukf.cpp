#include "estimators/ukf.hpp"

UKF::UKF(SEParameters se_parameters, std::shared_ptr<ProcessModel> process_model,
         std::shared_ptr<ObservationModel> observation_model)
    : params_(se_parameters),
      process_model_(process_model),
      observation_model_(observation_model),
      last_update_(rclcpp::Clock().now()) {
  // Initialize the process noise matrix, and observation noise
  // matrix
  process_noise_matrix_ = Eigen::Matrix<double, StateSize, StateSize>::Zero();
  measurement_noise_matrix_ = Eigen::Matrix<double, StateSize, StateSize>::Zero();

  lambda_ =
      se_parameters.alpha * se_parameters.alpha * (StateSize + se_parameters.kappa) - StateSize;

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

void UKF::compute_sigma_points(const State& state,
                               const Eigen::Matrix<double, StateSize, StateSize>& covariance,
                               Eigen::Matrix<double, 2 * StateSize + 1, StateSize>& sigma_points) {
  // Compute the square root of the covariance matrix using Cholesky decomposition.
  Eigen::MatrixXd scaled_covariance = (2 * StateSize + lambda_) * covariance;
  Eigen::LLT<Eigen::MatrixXd> llt(scaled_covariance);
  Eigen::MatrixXd sqrt_covariance = llt.matrixL();

  // Compute the actual sigma points.
  sigma_points.row(0) = state;
  for (int i = 0; i < StateSize; i++) {
    sigma_points.row(i + 1) = state + sqrt_covariance.col(i);
    sigma_points.row(i + 1 + StateSize) = state - sqrt_covariance.col(i);
  }

  return;
}

void UKF::get_mean(const Eigen::Matrix<double, 2 * StateSize + 1, StateSize>& sigma_points,
                   State& mean) {
  // Compute the mean of the sigma points using the weights.
  mean = Eigen::VectorXd::Zero(StateSize);
  for (int i = 0; i < 2 * StateSize + 1; i++) {
    mean += weights_(i) * sigma_points.row(i).transpose();
  }
}

void UKF::get_covariance(const Eigen::Matrix<double, 2 * StateSize + 1, StateSize>& sigma_points,
                         const State& mean,
                         Eigen::Matrix<double, StateSize, StateSize>& covariance) {
  // Compute the covariance of the sigma points using the weights.
  covariance = Eigen::MatrixXd::Zero(StateSize, StateSize);
  for (int i = 0; i < 2 * StateSize + 1; i++) {
    Eigen::VectorXd diff = sigma_points.row(i).transpose() - mean;
    covariance += weights_(i) * diff * diff.transpose();
  }
}