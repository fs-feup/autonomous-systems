#include "models/observation/pacsim_sensors.hpp"

void ObservationModelPacsim::expected_observations(const State& state,
                                                   Eigen::Ref<Eigen::VectorXd> expected) {
  Eigen::VectorXd& full = full_expected_;
  const double rad_to_rpm = 60 / (2 * M_PI);
  full(0) = state(AX) - state(YAW_RATE) * state(VY);
  full(1) = state(AY) + state(YAW_RATE) * state(VX);
  full(2) = state(YAW_RATE);
  full(11) = state(ST_ANGLE);

  Eigen::Vector4d wheel_angles = steering_model_->calculate_steering_angles(state(ST_ANGLE));

  double half_width = parameters_->car_parameters_->track_width / 2.0;
  double lr = parameters_->car_parameters_->cg_2_rear_axis;  // distance from CG to rear axle
  double lf = parameters_->car_parameters_->wheelbase - lr;  // distance from CG to front axle
  double wheel_radius = parameters_->car_parameters_->tire_parameters->effective_tire_r;

  Eigen::Vector4d x_pos = {lf, lf, -lr, -lr};
  Eigen::Vector4d y_pos = {half_width, -half_width, half_width, -half_width};

  for (int i = 0; i < 4; i++) {
    double v_x = state(VX) - state(YAW_RATE) * y_pos(i);
    double v_y = state(VY) + state(YAW_RATE) * x_pos(i);
    double v_lon = v_x * cos(wheel_angles(i)) + v_y * sin(wheel_angles(i));
    full(3 + i) = state(FL_WHEEL_SPEED + i) * rad_to_rpm;

    // Kinematic prediction without slip correction, or it collapses to the state-based
    // one and yaw rate loses observability.
    full(7 + i) = (v_lon / wheel_radius) * rad_to_rpm;
  }
  write_active(full, expected);
}

Eigen::VectorXd ObservationModelPacsim::get_last_observations() const {
  refresh_health();
  Eigen::VectorXd full(12);
  full << last_imu_data_.acceleration_x, last_imu_data_.acceleration_y,
      last_imu_data_.rotational_velocity, last_wss_data_.fl_rpm, last_wss_data_.fr_rpm,
      last_wss_data_.rl_rpm, last_wss_data_.rr_rpm, last_wss_data_.fl_rpm, last_wss_data_.fr_rpm,
      last_wss_data_.rl_rpm, last_wss_data_.rr_rpm, last_steering_angle_;
  return select_active(full);
}

Eigen::MatrixXd ObservationModelPacsim::get_last_observations_noise() const {
  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(12, 12);
  noise(0, 0) = parameters_->imu_acceleration_x_noise_;
  noise(1, 1) = parameters_->imu_acceleration_y_noise_;
  noise(2, 2) = parameters_->imu_rotational_noise_;
  noise(11, 11) = parameters_->steering_angle_noise_;

  // State-based and kinematic rows of a wheel share its physical measurement noise.
  // R_i = [[var*(1 + state_factor), cross_factor*var       ],
  //        [cross_factor*var,       var*(1 + kinematic_factor)]]
  const double var_wss = parameters_->wheel_speed_noise_;
  const double state_factor = parameters_->state_model_noise_factor_;
  const double kin_factor = parameters_->kinematic_model_noise_factor_;
  const double cross_factor = parameters_->cross_noise_factor_;
  for (int i = 0; i < 4; ++i) {
    const int idx_state = 3 + i;
    const int idx_kin = 7 + i;
    noise(idx_state, idx_state) = var_wss * (1.0 + state_factor);
    noise(idx_kin, idx_kin) = var_wss * (1.0 + kin_factor);
    noise(idx_state, idx_kin) = cross_factor * var_wss;
    noise(idx_kin, idx_state) = cross_factor * var_wss;
  }

  return finalize_noise(std::move(noise));
}

void ObservationModelPacsim::update_imu_data(const common_lib::sensor_data::ImuData& imu_data) {
  last_imu_data_ = imu_data;
  track(IMU, {imu_data.acceleration_x, imu_data.acceleration_y, imu_data.rotational_velocity},
        imu_data.timestamp_);
}

void ObservationModelPacsim::update_wss_data(
    const common_lib::sensor_data::WheelEncoderData& wss_data) {
  last_wss_data_ = wss_data;
  track(WSS_FL, {wss_data.fl_rpm}, wss_data.timestamp_);
  track(WSS_FR, {wss_data.fr_rpm}, wss_data.timestamp_);
  track(WSS_RL, {wss_data.rl_rpm}, wss_data.timestamp_);
  track(WSS_RR, {wss_data.rr_rpm}, wss_data.timestamp_);
}

void ObservationModelPacsim::update_steering_angle(double steering_angle,
                                                   const rclcpp::Time& stamp) {
  last_steering_angle_ = steering_angle;
  track(STEERING, {steering_angle}, stamp);
}

std::vector<SensorSpec> ObservationModelPacsim::build_specs(
    const std::shared_ptr<SEParameters>& parameters) {
  const double staleness_tol = parameters->sensor_staleness_miss_tolerance_;
  const double imu_timeout = staleness_tol / parameters->imu_frequency_;
  const double wheel_timeout = staleness_tol / parameters->wheel_speed_frequency_;
  const double steer_timeout = staleness_tol / parameters->steering_frequency_;

  const SignalSpec accel{-parameters->imu_accel_range_, parameters->imu_accel_range_,
                         parameters->imu_accel_max_rate_of_change_};
  const SignalSpec yaw{-parameters->imu_yaw_range_, parameters->imu_yaw_range_,
                       parameters->imu_yaw_max_rate_of_change_};
  const SignalSpec wheel{-parameters->wheel_speed_range_, parameters->wheel_speed_range_,
                         parameters->wheel_speed_max_rate_of_change_};
  const SignalSpec steer{-parameters->steering_range_, parameters->steering_range_,
                         parameters->steering_max_rate_of_change_};

  std::vector<SensorSpec> specs(NUM_SENSORS);
  specs[IMU] = {"imu", {accel, accel, yaw}, imu_timeout};
  specs[WSS_FL] = {"wss_fl", {wheel}, wheel_timeout};
  specs[WSS_FR] = {"wss_fr", {wheel}, wheel_timeout};
  specs[WSS_RL] = {"wss_rl", {wheel}, wheel_timeout};
  specs[WSS_RR] = {"wss_rr", {wheel}, wheel_timeout};
  specs[STEERING] = {"steering", {steer}, steer_timeout};
  return specs;
}

std::vector<std::size_t> ObservationModelPacsim::build_row_to_sensor() {
  return {IMU, IMU, IMU, WSS_FL, WSS_FR, WSS_RL, WSS_RR, WSS_FL, WSS_FR, WSS_RL, WSS_RR, STEERING};
}
