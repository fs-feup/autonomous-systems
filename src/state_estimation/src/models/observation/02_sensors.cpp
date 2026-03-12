#include "models/observation/02_sensors.hpp"

void ObservationModel02::expected_observations(const State& state,
                                               Eigen::Ref<Eigen::VectorXd> expected_observations) {
  // Basically mapping of state except for wheel speeds which are converted from rad/s to rpm and
  // motor torque which is calculated.
  expected_observations.resize(9);
  double rad_to_rpm = 60 / (2 * M_PI);
  double average_rear_wheel_rpm =
      ((state(RL_WHEEL_SPEED) + state(RR_WHEEL_SPEED)) / 2) * rad_to_rpm;
  expected_observations << state(AX), state(AY), state(YAW_RATE),
      state(FL_WHEEL_SPEED) * rad_to_rpm, state(FR_WHEEL_SPEED) * rad_to_rpm,
      state(RL_WHEEL_SPEED) * rad_to_rpm, state(RR_WHEEL_SPEED) * rad_to_rpm, state(ST_ANGLE),
      average_rear_wheel_rpm * this->parameters_->car_parameters_->gear_ratio;
}

Eigen::VectorXd ObservationModel02::get_last_observations() const {
  Eigen::VectorXd last_observations(9);
  last_observations << last_imu_data_.acceleration_x, last_imu_data_.acceleration_y,
      last_imu_data_.rotational_velocity, last_wss_data_.fl_rpm, last_wss_data_.fr_rpm,
      last_wss_data_.rl_rpm, last_wss_data_.rr_rpm, last_steering_angle_, last_motor_rpm_;
  return last_observations;
}

Eigen::MatrixXd ObservationModel02::get_last_observations_noise() const {
  // TODO: Measurement noise matrix should be based on the number of active sensors
  Eigen::MatrixXd noise_matrix = Eigen::MatrixXd::Zero(9, 9);
  noise_matrix(0, 0) = this->parameters_->imu_acceleration_x_noise_;
  noise_matrix(1, 1) = this->parameters_->imu_acceleration_y_noise_;
  noise_matrix(2, 2) = this->parameters_->imu_rotational_noise_;
  noise_matrix(3, 3) = this->parameters_->wheel_speed_noise_;
  noise_matrix(4, 4) = this->parameters_->wheel_speed_noise_;
  noise_matrix(5, 5) = this->parameters_->wheel_speed_noise_;
  noise_matrix(6, 6) = this->parameters_->wheel_speed_noise_;
  noise_matrix(7, 7) = this->parameters_->steering_angle_noise_;
  noise_matrix(8, 8) = this->parameters_->motor_rpm_noise_;
  return noise_matrix;
}