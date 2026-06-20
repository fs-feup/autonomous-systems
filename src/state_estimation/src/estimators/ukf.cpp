#include "estimators/ukf.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

#include <algorithm>

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

  // Precomputed constant used in every covariance update
  w0_cov_correction_ = 1.0 + se_parameters->beta_ - alpha2;

  // Build one process-model instance per prediction thread 
#ifdef _OPENMP
  prediction_threads_ = std::max(1, se_parameters->prediction_threads_);
#else
  prediction_threads_ = 1;
  RCLCPP_WARN_STREAM(rclcpp::get_logger("state_estimation"),
                      "OpenMP is not enabled, prediction_threads_ set to 1.");
#endif
  process_model_pool_.resize(prediction_threads_);
  process_model_pool_[0] = process_model_;
  for (int i = 1; i < prediction_threads_; ++i) {
    process_model_pool_[i] = process_models_map.at(se_parameters->process_model_name_)(se_parameters);
  }

  // Cache measurement size and preallocate all workspace matrices to eliminate per-callback heap
  // allocations, which are the primary cause of execution-time variance
  meas_size_ = observation_model->get_measurement_size();
  predicted_measurements_.resize(2 * StateSize + 1, meas_size_);
  centered_measurements_.resize(2 * StateSize + 1, meas_size_);
  cross_covariance_.resize(StateSize, meas_size_);
  kalman_gain_.resize(StateSize, meas_size_);
  predicted_measurement_covariance_.resize(meas_size_, meas_size_);
  H_.resize(meas_size_, StateSize);
  predicted_measurement_mean_.resize(meas_size_);
  G_.resize(StateSize, StateSize);
}

void UKF::compute_sigma_points(
    const State& state, const Eigen::Matrix<double, StateSize, StateSize>& covariance,
    Eigen::Matrix<double, 2 * StateSize + 1, StateSize, Eigen::RowMajor>& sigma_points) {
      
  Eigen::LLT<Eigen::Matrix<double, StateSize, StateSize>> llt(StateSize * covariance);
  const Eigen::Matrix<double, StateSize, StateSize> sqrt_covariance = llt.matrixL();

  sigma_points.row(0) = state;
  for (int i = 0; i < StateSize; i++) {
    sigma_points.row(i + 1) = state + params_->alpha_ * sqrt_covariance.col(i);
    sigma_points.row(i + 1 + StateSize) = state - params_->alpha_ * sqrt_covariance.col(i);
  }
}

void UKF::control_callback(const common_lib::structures::ControlCommand& control_command) {
  std::lock_guard<std::mutex> lock(control_command_mutex_);
  this->last_control_command_ = control_command;
}

void UKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_imu_data(imu_data);
}

void UKF::wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_wss_data(wss_data);
}

void UKF::motor_rpm_callback(double motor_rpm, const rclcpp::Time& stamp) {
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_motor_rpm(motor_rpm, stamp);
}

void UKF::steering_callback(double steering_angle, const rclcpp::Time& stamp) {
  std::lock_guard<std::mutex> lock(observation_model_mutex_);
  this->observation_model_->update_steering_angle(steering_angle, stamp);
}

void UKF::timer_callback(State& curr_state) {
  // Measure timing overhead (input gathering, locking, etc.)
  auto start_time = std::chrono::high_resolution_clock::now();

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

  // Predict Step
  auto prediction_start = std::chrono::high_resolution_clock::now();

  compute_sigma_points(state_, covariance_, sigma_points_);

  // Predict the sigma points through the process model.
  #pragma omp parallel for schedule(static) num_threads(prediction_threads_)
  for (int i = 0; i < sigma_points_.rows(); ++i) {
    #ifdef _OPENMP
      const int tid = omp_get_thread_num();
    #else
      const int tid = 0;
    #endif
    process_model_pool_[tid]->predict(sigma_points_.row(i), control_command, dt);
  }

  // Compute the predicted mean and covariance of the state
  State predicted_state = sigma_points_.transpose() * weights_m_;

  centered_ = sigma_points_.rowwise() - predicted_state.transpose();
  Eigen::Matrix<double, StateSize, StateSize> predicted_covariance =
      centered_.transpose() * weights_m_.asDiagonal() * centered_ +
      w0_cov_correction_ * (sigma_points_.row(0).transpose() - predicted_state) *
          (sigma_points_.row(0).transpose() - predicted_state).transpose() +
      process_noise_matrix_;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Predicted State: \n"
                                                                  << predicted_state);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Predicted Covariance: \n"
                                                                  << predicted_covariance);

  // Correction Step
  auto correction_start = std::chrono::high_resolution_clock::now();

  // Support runtime measurement-size changes
  {
    int current_meas_size = observation_model_->get_measurement_size();
    if (current_meas_size != meas_size_) {
      meas_size_ = current_meas_size;
      predicted_measurements_.resize(2 * StateSize + 1, meas_size_);
      centered_measurements_.resize(2 * StateSize + 1, meas_size_);
      cross_covariance_.resize(StateSize, meas_size_);
      kalman_gain_.resize(StateSize, meas_size_);
      predicted_measurement_covariance_.resize(meas_size_, meas_size_);
      H_.resize(meas_size_, StateSize);
      predicted_measurement_mean_.resize(meas_size_);
    }
  }

  // Predict the measurements for each sigma point
  for (int i = 0; i < 2 * StateSize + 1; i++) {
    observation_model_->expected_observations(sigma_points_.row(i), predicted_measurements_.row(i));
  }

  // Compute the mean and covariance of the predicted measurements
  predicted_measurement_mean_ = predicted_measurements_.transpose() * weights_m_;
  centered_measurements_ =
      predicted_measurements_.rowwise() - predicted_measurement_mean_.transpose();
  predicted_measurement_covariance_ =
      centered_measurements_.transpose() * weights_m_.asDiagonal() * centered_measurements_ +
      w0_cov_correction_ *
          (predicted_measurements_.row(0).transpose() - predicted_measurement_mean_) *
          (predicted_measurements_.row(0).transpose() - predicted_measurement_mean_).transpose() +
      last_observation_noise;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Last Observation: \n"
                                                                  << last_observation);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Predicted Measurement: \n"
                                                                  << predicted_measurement_mean_);
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("state_estimation"), "2 - Covariance: \n" <<
  // predicted_measurement_covariance_);

  // Update Step
  auto update_start = std::chrono::high_resolution_clock::now();

  // Compute the cross covariance
  cross_covariance_ =
      centered_.transpose() * weights_m_.asDiagonal() * centered_measurements_ +
      w0_cov_correction_ * (sigma_points_.row(0).transpose() - predicted_state) *
          (predicted_measurements_.row(0).transpose() - predicted_measurement_mean_).transpose();

  // Compute Kalman gain
  kalman_gain_ =
      predicted_measurement_covariance_.ldlt().solve(cross_covariance_.transpose()).transpose();

  // Update the state and covariance
  State updated_state =
      predicted_state + kalman_gain_ * (last_observation - predicted_measurement_mean_);
  Eigen::Matrix<double, StateSize, StateSize> updated_covariance;

  // Derive the 'Pseudo-H' for the Joseph Form: Pxz = P * H'  →  H = (P^{-1} * Pxz)'
  H_ = (predicted_covariance.ldlt().solve(cross_covariance_)).transpose();

  // Joseph Form: P = (I - KH) * P_pred * (I - KH)' + K * R * K'
  // Numerically guaranteed to be positive-definite
  G_ = Eigen::Matrix<double, StateSize, StateSize>::Identity() - kalman_gain_ * H_;
  updated_covariance = G_ * predicted_covariance * G_.transpose() +
                       kalman_gain_ * last_observation_noise * kalman_gain_.transpose();

  updated_covariance = 0.5 * (updated_covariance + updated_covariance.transpose());
  updated_covariance.diagonal().array() += 1e-9;

  state_ = updated_state;
  covariance_ = updated_covariance;
  last_update_ = now;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "Updated State: \n" << updated_state);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("state_estimation"), "3 - Covariance: \n"
                                                                  << updated_covariance);
  curr_state = state_;
  auto end_time = std::chrono::high_resolution_clock::now();

  if (params_->publish_vm_debug_info_) {
    this->process_model_data_ = process_model_->get_process_model_data(state_, control_command);
  }

  // Store execution times for each stage in milliseconds
  // [0] = input gathering + overhead (setup to prediction start)
  // [1] = prediction stage (sigma points, prediction, state mean/covariance)
  // [2] = correction stage (measurement prediction, measurement mean/covariance)
  // [3] = update stage (cross-covariance, Kalman gain, state update, process model data)
  execution_times_(0) =
      std::chrono::duration<double, std::milli>(prediction_start - start_time).count();
  execution_times_(1) =
      std::chrono::duration<double, std::milli>(correction_start - prediction_start).count();
  execution_times_(2) =
      std::chrono::duration<double, std::milli>(update_start - correction_start).count();
  execution_times_(3) = std::chrono::duration<double, std::milli>(end_time - update_start).count();

  //if(state_(VY) > 0.1 || state_(VY) < -0.1){
  //  RCLCPP_WARN_STREAM(rclcpp::get_logger("state_estimation"), "Lateral velocity is high: " << state_(VY));
  //  RCLCPP_INFO_STREAM(rclcpp::get_logger("state_estimation"),
  //                    "Time since last update: " << dt << " seconds");
  //  RCLCPP_INFO_STREAM(rclcpp::get_logger("state_estimation"), "Updated State: \n" << updated_state);
  //  RCLCPP_INFO_STREAM(rclcpp::get_logger("state_estimation"), "3 - Covariance: \n"
  //                                                                << updated_covariance);
  //}
}