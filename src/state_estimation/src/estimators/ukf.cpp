#include "estimators/ukf.hpp"

// TODO: Measurement noise matrix with an i number of measurements

UKF::UKF(std::shared_ptr<SEParameters> se_parameters, std::shared_ptr<ProcessModel> process_model,
         std::shared_ptr<ObservationModel> observation_model)
    : params_(se_parameters),
      process_model_(process_model),
      observation_model_(observation_model),
      last_update_(rclcpp::Time(0)) {
  // Compute sigma point weights
  double alpha2 = se_parameters->alpha_ * se_parameters->alpha_;
  weights_m_ = Eigen::VectorXd(2 * StateSize + 1);
  weights_m_(0) = se_parameters->mean_weight_ / alpha2 + (1 - 1 / alpha2);
  for (int i = 1; i < 2 * StateSize + 1; i++) {
    weights_m_(i) = ((1 - se_parameters->mean_weight_) / (2 * (StateSize)) / alpha2);
  }

  // Initialize the process noise matrix
  process_noise_matrix_ = Eigen::Matrix<double, StateSize, StateSize>::Zero();
  process_noise_matrix_(VX, VX) = se_parameters->velocity_x_process_noise_;
  process_noise_matrix_(VY, VY) = se_parameters->velocity_y_process_noise_;
  process_noise_matrix_(YAW_RATE, YAW_RATE) = se_parameters->yaw_rate_process_noise_;
  process_noise_matrix_(AX, AX) = se_parameters->acceleration_x_process_noise_;
  process_noise_matrix_(AY, AY) = se_parameters->acceleration_y_process_noise_;
  process_noise_matrix_(ST_ANGLE, ST_ANGLE) = se_parameters->steering_angle_process_noise_;
  process_noise_matrix_(FL_WHEEL_SPEED, FL_WHEEL_SPEED) = se_parameters->wheel_speed_process_noise_;
  process_noise_matrix_(FR_WHEEL_SPEED, FR_WHEEL_SPEED) = se_parameters->wheel_speed_process_noise_;
  process_noise_matrix_(RL_WHEEL_SPEED, RL_WHEEL_SPEED) = se_parameters->wheel_speed_process_noise_;
  process_noise_matrix_(RR_WHEEL_SPEED, RR_WHEEL_SPEED) = se_parameters->wheel_speed_process_noise_;
}

void UKF::compute_sigma_points(
    const State& state, const Eigen::Matrix<double, StateSize, StateSize>& covariance,
    Eigen::Matrix<double, 2 * StateSize + 1, StateSize, Eigen::RowMajor>& sigma_points) {
  // Compute the square root of the covariance matrix using Cholesky decomposition.
  Eigen::MatrixXd scaled_covariance = StateSize * covariance;
  Eigen::LLT<Eigen::MatrixXd> llt(scaled_covariance);
  Eigen::MatrixXd sqrt_covariance = llt.matrixL();

  // Compute the actual sigma points.
  sigma_points.row(0) = state;
  for (int i = 0; i < StateSize; i++) {
    sigma_points.row(i + 1) = state + params_->alpha_ * (sqrt_covariance.col(i));
    sigma_points.row(i + 1 + StateSize) = state - params_->alpha_ * (sqrt_covariance.col(i));
  }

  return;
}

void UKF::control_callback(const common_lib::structures::ControlCommand& control_command) {
  std::lock_guard<std::mutex> lock(control_command_mutex_);
  this->last_control_command_ = control_command;
}

void UKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  // TODO: Logic for broken sensor or data
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_imu_data(imu_data);
}

void UKF::wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  // TODO: Logic for broken sensor or data
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_wss_data(wss_data);
}

void UKF::motor_rpm_callback(double motor_rpm) {
  // TODO: Logic for broken sensor or data
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_motor_rpm(motor_rpm);
}

void UKF::steering_callback(double steering_angle) {
  // TODO: Logic for broken sensor or data
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_steering_angle(steering_angle);
}

void UKF::timer_callback(State& curr_state) {
  RCLCPP_DEBUG(
      rclcpp::get_logger("state_estimation"),
      "Timer callback triggered for state estimation update.------------------------------------");

  rclcpp::Time now = rclcpp::Clock().now();
  double dt = (now - last_update_).seconds();

  if (last_update_.nanoseconds() == 0) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("state_estimation"), "First update, skipping.");
    last_update_ = now;
    curr_state = state_;
    return;
  }

  // Lock inputs
  Eigen::VectorXd last_observation;
  Eigen::MatrixXd last_observation_noise;
  common_lib::structures::ControlCommand control_command;

  {
    std::lock_guard<std::mutex> lock(observation_model_mutex_);
    last_observation = this->observation_model_->get_last_observations();
    last_observation_noise = this->observation_model_->get_last_observations_noise();
  }

  {
    std::lock_guard<std::mutex> lock(control_command_mutex_);
    control_command = this->last_control_command_;
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"),
                      "Time since last update: " << dt << " seconds");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Current State: \n" << state_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"),
                      "Control Command: \n"
                          << "Throttle FL: " << control_command.throttle_fl
                          << ", Throttle FR: " << control_command.throttle_fr
                          << ", Throttle RL: " << control_command.throttle_rl
                          << ", Throttle RR: " << control_command.throttle_rr
                          << ", Steering Angle: " << control_command.steering_angle);

  // Prediction step

  Eigen::Matrix<double, 2 * StateSize + 1, StateSize, Eigen::RowMajor> sigma_points;
  compute_sigma_points(state_, covariance_, sigma_points);

  // Predict the sigma points through the process model
  for (int i = 0; i < sigma_points.rows(); ++i) {
    process_model_->predict(sigma_points.row(i), control_command, dt);
  }

  // Compute the predicted mean and covariance of the state
  State predicted_state = sigma_points.transpose() * weights_m_;

  Eigen::MatrixXd centered = sigma_points.rowwise() - predicted_state.transpose();
  Eigen::Matrix<double, StateSize, StateSize> predicted_covariance =
      centered.transpose() * weights_m_.asDiagonal() * centered +
      (1 + params_->beta_ - params_->alpha_ * params_->alpha_) *
          (sigma_points.row(0).transpose() - predicted_state) *
          (sigma_points.row(0).transpose() - predicted_state).transpose() +
      process_noise_matrix_;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Predicted State: \n"
                                                                  << predicted_state);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Predicted Covariance: \n"
                                                                  << predicted_covariance);

  // Correction step

  // Predict the measurements for each sigma point
  Eigen::Matrix<double, 2 * StateSize + 1, Eigen::Dynamic, Eigen::RowMajor> predicted_measurements =
      Eigen::MatrixXd::Zero(2 * StateSize + 1, observation_model_->get_measurement_size());
  for (int i = 0; i < 2 * StateSize + 1; i++) {
    observation_model_->expected_observations(sigma_points.row(i), predicted_measurements.row(i));
  }

  // Compute the mean and covariance of the predicted measurements
  Eigen::VectorXd predicted_measurement_mean = predicted_measurements.transpose() * weights_m_;
  Eigen::MatrixXd centered_measurements =
      predicted_measurements.rowwise() - predicted_measurement_mean.transpose();
  Eigen::MatrixXd predicted_measurement_covariance =
      centered_measurements.transpose() * weights_m_.asDiagonal() * centered_measurements +
      (1 + params_->beta_ - params_->alpha_ * params_->alpha_) *
          (predicted_measurements.row(0).transpose() - predicted_measurement_mean) *
          (predicted_measurements.row(0).transpose() - predicted_measurement_mean).transpose() +
      last_observation_noise;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Last Observation: \n"
                                                                  << last_observation);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Predicted Measurement: \n"
                                                                  << predicted_measurement_mean);
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("state_estimation"), "2 - Covariance: \n" <<
  // predicted_measurement_covariance);

  // Compute the cross covariance between the state and the measurements (State_Errors)^T *
  // Diag(Weights) * (Meas_Errors)
  Eigen::MatrixXd state_centered = sigma_points.rowwise() - predicted_state.transpose();
  Eigen::MatrixXd meas_centered =
      predicted_measurements.rowwise() - predicted_measurement_mean.transpose();
  Eigen::MatrixXd cross_covariance =
      state_centered.transpose() * weights_m_.asDiagonal() * meas_centered +
      (1 + params_->beta_ - params_->alpha_ * params_->alpha_) *
          (sigma_points.row(0).transpose() - predicted_state) *
          (predicted_measurements.row(0).transpose() - predicted_measurement_mean).transpose();

  // Compute the Kalman gain
  Eigen::MatrixXd kalman_gain = cross_covariance * predicted_measurement_covariance.inverse();

  // Update the state and covariance
  State updated_state =
      predicted_state + kalman_gain * (last_observation - predicted_measurement_mean);
  Eigen::Matrix<double, StateSize, StateSize> updated_covariance;
  // 1. Setup
  Eigen::Matrix<double, StateSize, StateSize> I =
      Eigen::Matrix<double, StateSize, StateSize>::Identity();

  // 2. Derive the 'Pseudo-H' for the Joseph Form
  // Since Pxz = P * H', then H = (P^-1 * Pxz)'
  Eigen::MatrixXd H = (predicted_covariance.ldlt().solve(cross_covariance)).transpose();

  // 3. The Joseph Form term: (I - KH)
  Eigen::MatrixXd K = kalman_gain;
  Eigen::MatrixXd G = I - K * H;

  // 4. Calculate P = (G * P_pred * G') + (K * R * K')
  // This form is numerically guaranteed to be positive-definite
  updated_covariance =
      G * predicted_covariance * G.transpose() + K * last_observation_noise * K.transpose();

  // 5. Final pass for symmetry and numerical health
  updated_covariance = 0.5 * (updated_covariance + updated_covariance.transpose());
  updated_covariance.diagonal().array() += 1e-9;

  state_ = updated_state;
  covariance_ = updated_covariance;
  last_update_ = now;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Updated State: \n" << updated_state);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "3 - Covariance: \n"
                                                                  << updated_covariance);
  curr_state = state_;
  if (params_->publish_vm_debug_info_) {
    this->process_model_data_ = process_model_->get_process_model_data(state_, control_command);
  }
}