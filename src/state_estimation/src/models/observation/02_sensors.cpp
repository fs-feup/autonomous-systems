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
      average_rear_wheel_rpm * this->parameters_->car_parameters_.gear_ratio;
}