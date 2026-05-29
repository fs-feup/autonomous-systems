#include "motion_lib/transmission_model/open_differential.hpp"

#include <algorithm>
#include <cmath>

double OpenDifferential::calculate_motor_omega(
    const common_lib::structures::Wheels& wheel_speeds) const {
  double avg_rear_speed = (wheel_speeds.rear_left + wheel_speeds.rear_right) / 2.0;
  return avg_rear_speed * car_parameters_->transmission_parameters->gear_ratio;
}

common_lib::structures::Wheels OpenDifferential::calculate_wheel_torques(
    double input_torque, const common_lib::structures::Wheels& wheel_speeds) const {
  const auto& p = car_parameters_->transmission_parameters;

  double motor_omega = calculate_motor_omega(wheel_speeds);

  // Convert shaft torque to axle torque accounting for efficiency and gear ratio
  double axle_torque = input_torque * p->efficiency * p->gear_ratio;

  common_lib::structures::Wheels torques;
  torques.front_left = 0.0;
  torques.front_right = 0.0;
  torques.rear_left = axle_torque / 2.0;
  torques.rear_right = axle_torque / 2.0;

  return torques;
}
