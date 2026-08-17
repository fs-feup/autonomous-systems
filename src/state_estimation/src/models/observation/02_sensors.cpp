#include "models/observation/02_sensors.hpp"

#include <array>

void ObservationModel02::expected_observations(const State& state,
                                               Eigen::Ref<Eigen::VectorXd> expected) {
  Eigen::VectorXd& full = full_expected_;
  const double rad_to_rpm = 60 / (2 * M_PI);
  
  full(0) = state(AX) - state(YAW_RATE) * state(VY);
  full(1) = state(AY) + state(YAW_RATE) * state(VX);
  full(2) = state(YAW_RATE);
  full(7) = state(ST_ANGLE);

  const double half_width = parameters_->car_parameters_->track_width / 2.0;
  const double lr = parameters_->car_parameters_->cg_2_rear_axis;  // CG to rear axle
  const double lf = parameters_->car_parameters_->wheelbase - lr;  // CG to front axle
  const double wheel_radius = parameters_->car_parameters_->tire_parameters->effective_tire_r;
  const double gear_ratio = parameters_->car_parameters_->gear_ratio;

  // State-based front wheel predictions: observe the wheel-speed states directly.
  full(3) = state(FL_WHEEL_SPEED) * rad_to_rpm;
  full(4) = state(FR_WHEEL_SPEED) * rad_to_rpm;

  // Kinematic front wheel speed.
  const Eigen::Vector4d wheel_angles = steering_model_->calculate_steering_angles(state(ST_ANGLE));
  const Eigen::Vector2d x_pos = {lf, lf};
  const Eigen::Vector2d y_pos = {half_width, -half_width};
  for (int i = 0; i < 2; ++i) {
    const double v_x = state(VX) - state(YAW_RATE) * y_pos(i);
    const double v_y = state(VY) + state(YAW_RATE) * x_pos(i);
    const double v_lon = v_x * cos(wheel_angles(i)) + v_y * sin(wheel_angles(i));
    full(5 + i) = (v_lon / wheel_radius) * rad_to_rpm;
  }

  // RWD motor, state-based and kinematic.
  const double avg_rear_state_rpm =
      ((state(RL_WHEEL_SPEED) + state(RR_WHEEL_SPEED)) / 2.0) * rad_to_rpm;
  full(8) = avg_rear_state_rpm * gear_ratio;

  double rear_kin_sum = 0.0;
  for (int i = 0; i < 2; ++i) {
    const double v_x = state(VX) - state(YAW_RATE) * y_pos(i);
    rear_kin_sum += (v_x / wheel_radius) * rad_to_rpm;
  }
  full(9) = (rear_kin_sum / 2.0) * gear_ratio;

  write_active(full, expected);
}

Eigen::VectorXd ObservationModel02::get_last_observations() const {
  refresh_health();
  // Each wheel/motor sensor value feeds both its state-based and kinematic rows.
  Eigen::VectorXd full(10);
  full << last_imu_data_.acceleration_x, last_imu_data_.acceleration_y,
      last_imu_data_.rotational_velocity, last_wss_data_.fl_rpm, last_wss_data_.fr_rpm,
      last_wss_data_.fl_rpm, last_wss_data_.fr_rpm, last_steering_angle_, last_motor_rpm_,
      last_motor_rpm_;
  return select_active(full);
}

Eigen::MatrixXd ObservationModel02::get_last_observations_noise() const {
  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(10, 10);
  noise(0, 0) = parameters_->imu_acceleration_x_noise_;
  noise(1, 1) = parameters_->imu_acceleration_y_noise_;
  noise(2, 2) = parameters_->imu_rotational_noise_;
  noise(7, 7) = parameters_->steering_angle_noise_;

  // A sensor's state-based and kinematic rows read the same physical measurement, so
  // they share its noise with an off-diagonal correlation. Pairs: {state row, kinematic row}.
  // R_pair = [[var*(1 + state_factor), cross_factor*var       ],
  //           [cross_factor*var,       var*(1 + kinematic_factor)]]
  const double state_factor = parameters_->state_model_noise_factor_;
  const double kin_factor = parameters_->kinematic_model_noise_factor_;
  const double cross_factor = parameters_->cross_noise_factor_;
  const double var_wss = parameters_->wheel_speed_noise_;
  const double var_motor = parameters_->motor_rpm_noise_;
  const std::array<std::pair<int, int>, 2> front_pairs = {{{3, 5}, {4, 6}}};  // FL, FR
  for (const auto& [idx_state, idx_kin] : front_pairs) {
    noise(idx_state, idx_state) = var_wss * (1.0 + state_factor);
    noise(idx_kin, idx_kin) = var_wss * (1.0 + kin_factor);
    noise(idx_state, idx_kin) = cross_factor * var_wss;
    noise(idx_kin, idx_state) = cross_factor * var_wss;
  }
  noise(8, 8) = var_motor * (1.0 + state_factor);
  noise(9, 9) = var_motor * (1.0 + kin_factor);
  noise(8, 9) = cross_factor * var_motor;
  noise(9, 8) = cross_factor * var_motor;

  return finalize_noise(std::move(noise));
}

void ObservationModel02::update_imu_data(const common_lib::sensor_data::ImuData& imu_data) {
  last_imu_data_ = imu_data;
  track(IMU, {imu_data.acceleration_x, imu_data.acceleration_y, imu_data.rotational_velocity},
        imu_data.timestamp_);
}

void ObservationModel02::update_wss_data(
    const common_lib::sensor_data::WheelEncoderData& wss_data) {
  last_wss_data_ = wss_data;
  // 02 has only front wheel-speed sensors; the rear rpm fields are not used.
  track(WSS_FL, {wss_data.fl_rpm}, wss_data.timestamp_);
  track(WSS_FR, {wss_data.fr_rpm}, wss_data.timestamp_);
}

void ObservationModel02::update_motor_rpm(double motor_rpm, const rclcpp::Time& stamp) {
  last_motor_rpm_ = motor_rpm;
  track(MOTOR, {motor_rpm}, stamp);
}

void ObservationModel02::update_steering_angle(double steering_angle, const rclcpp::Time& stamp) {
  last_steering_angle_ = steering_angle;
  track(STEERING, {steering_angle}, stamp);
}

std::vector<SensorSpec> ObservationModel02::build_specs(
    const std::shared_ptr<SEParameters>& parameters) {
  const double staleness_tol = parameters->sensor_staleness_miss_tolerance_;
  const SignalSpec accel{-parameters->imu_accel_range_, parameters->imu_accel_range_,
                         parameters->imu_accel_max_rate_of_change_};
  const SignalSpec yaw{-parameters->imu_yaw_range_, parameters->imu_yaw_range_,
                       parameters->imu_yaw_max_rate_of_change_};
  const SignalSpec wheel{-parameters->wheel_speed_range_, parameters->wheel_speed_range_,
                         parameters->wheel_speed_max_rate_of_change_};
  const SignalSpec steer{-parameters->steering_range_, parameters->steering_range_,
                         parameters->steering_max_rate_of_change_};
  const SignalSpec motor{-parameters->motor_rpm_range_, parameters->motor_rpm_range_,
                         parameters->motor_rpm_max_rate_of_change_};

  std::vector<SensorSpec> specs(NUM_SENSORS);
  specs[IMU] = {"imu", {accel, accel, yaw}, staleness_tol / parameters->imu_frequency_};
  specs[WSS_FL] = {"wss_fl", {wheel}, staleness_tol / parameters->wheel_speed_frequency_};
  specs[WSS_FR] = {"wss_fr", {wheel}, staleness_tol / parameters->wheel_speed_frequency_};
  specs[STEERING] = {"steering", {steer}, staleness_tol / parameters->steering_frequency_};
  specs[MOTOR] = {"motor", {motor}, staleness_tol / parameters->motor_frequency_};
  return specs;
}

std::vector<std::size_t> ObservationModel02::build_row_to_sensor() {
  return {IMU, IMU, IMU, WSS_FL, WSS_FR, WSS_FL, WSS_FR, STEERING, MOTOR, MOTOR};
}
