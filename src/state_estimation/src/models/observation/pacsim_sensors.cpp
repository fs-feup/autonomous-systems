#include "models/observation/pacsim_sensors.hpp"

void ObservationModelPacsim::expected_observations(
    const State& state, Eigen::Ref<Eigen::VectorXd> expected_observations) {
  expected_observations.resize(12);
  double rad_to_rpm = 60 / (2 * M_PI);
  expected_observations(0) = state(AX);
  expected_observations(1) = state(AY);
  expected_observations(2) = state(YAW_RATE);
  expected_observations(11) = state(ST_ANGLE);

  Eigen::Vector4d wheel_angles = steering_model_->calculate_steering_angles(state(ST_ANGLE));

  // Calculate wheel speeds
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
    // State-based wheel angular speed prediction.
    expected_observations(3 + i) = state(FL_WHEEL_SPEED + i) * rad_to_rpm;

    // Kinematic wheel angular speed prediction (pure kinematics — no slip correction here,
    // or the prediction collapses to the state-based one and yaw rate loses observability).
    expected_observations(7 + i) = (v_lon / wheel_radius) * rad_to_rpm;
  }
}

Eigen::VectorXd ObservationModelPacsim::get_last_observations() const {
  Eigen::VectorXd last_observations(12);
  last_observations << last_imu_data_.acceleration_x, last_imu_data_.acceleration_y,
      last_imu_data_.rotational_velocity, last_wss_data_.fl_rpm, last_wss_data_.fr_rpm,
      last_wss_data_.rl_rpm, last_wss_data_.rr_rpm, last_wss_data_.fl_rpm, last_wss_data_.fr_rpm,
      last_wss_data_.rl_rpm, last_wss_data_.rr_rpm, last_steering_angle_;
  return last_observations;
}

Eigen::MatrixXd ObservationModelPacsim::get_last_observations_noise() const {
  // TODO: Measurement noise matrix should be based on the number of active sensors
  Eigen::MatrixXd noise_matrix = Eigen::MatrixXd::Zero(12, 12);
  noise_matrix(0, 0) = this->parameters_->imu_acceleration_x_noise_;
  noise_matrix(1, 1) = this->parameters_->imu_acceleration_y_noise_;
  noise_matrix(2, 2) = this->parameters_->imu_rotational_noise_;

  // Wheel-speed duplicated constraints share the same physical WSS sensor noise.
  // R_i = [[var_wss + var_state_model, var_wss],
  //        [var_wss,                   var_wss + var_kin_model]]
  const double var_wss = this->parameters_->wheel_speed_noise_;
  const double var_state_model = 0.25 * var_wss;
  const double var_kin_model = 6.0 * var_wss;
  for (int i = 0; i < 4; ++i) {
    const int idx_state = 3 + i;
    const int idx_kin = 7 + i;
    noise_matrix(idx_state, idx_state) = var_wss + var_state_model;
    noise_matrix(idx_kin, idx_kin) = var_wss + var_kin_model;
    noise_matrix(idx_state, idx_kin) = var_wss;
    noise_matrix(idx_kin, idx_state) = var_wss;
  }

  noise_matrix(11, 11) = this->parameters_->steering_angle_noise_;
  return noise_matrix;
}

void ObservationModelPacsim::update_imu_data(const common_lib::sensor_data::ImuData& imu_data) {
  last_imu_data_ = imu_data;
}

void ObservationModelPacsim::update_wss_data(
    const common_lib::sensor_data::WheelEncoderData& wss_data) {
  last_wss_data_ = wss_data;
}

void ObservationModelPacsim::update_steering_angle(double steering_angle) {
  last_steering_angle_ = steering_angle;
}

int ObservationModelPacsim::get_measurement_size() const { return 12; }