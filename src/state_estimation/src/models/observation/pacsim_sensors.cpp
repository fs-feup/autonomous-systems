#include "models/observation/pacsim_sensors.hpp"

void ObservationModelPacsim::expected_observations(
    const State& state, Eigen::Ref<Eigen::VectorXd>& expected_observations) {
  // Basically mapping of state except for wheel speeds which are converted from rad/s to rpm.
  expected_observations.resize(8);
  double rad_to_rpm = 60 / (2 * M_PI);
  expected_observations << state(AX), state(AY), state(YAW_RATE),
      state(FL_WHEEL_SPEED) * rad_to_rpm, state(FR_WHEEL_SPEED) * rad_to_rpm,
      state(RL_WHEEL_SPEED) * rad_to_rpm, state(RR_WHEEL_SPEED) * rad_to_rpm, state(ST_ANGLE);
}

Eigen::VectorXd ObservationModelPacsim::get_last_observations() const {
  Eigen::VectorXd last_observations(8);
  last_observations << last_imu_data_.acceleration_x, last_imu_data_.acceleration_y,
      last_imu_data_.rotational_velocity, last_wss_data_.fl_rpm, last_wss_data_.fr_rpm,
      last_wss_data_.rl_rpm, last_wss_data_.rr_rpm, last_steering_angle_;
  return last_observations;
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

int ObservationModelPacsim::get_measurement_size() const { return 8; }