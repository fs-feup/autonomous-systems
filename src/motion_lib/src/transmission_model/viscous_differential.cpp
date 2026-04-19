#include "motion_lib/transmission_model/viscous_differential.hpp"

#include <algorithm>
#include <cmath>

double ViscousDifferential::calculate_motor_omega(
    const common_lib::structures::Wheels& wheel_speeds) const {
  double avg_rear_speed = (wheel_speeds.rear_left + wheel_speeds.rear_right) / 2.0;
  return avg_rear_speed * car_parameters_->transmission_parameters->gear_ratio;
}

common_lib::structures::Wheels ViscousDifferential::calculate_wheel_torques(
    double input_torque, const common_lib::structures::Wheels& wheel_speeds) const {
  const auto& p = car_parameters_->transmission_parameters;

  double motor_omega = calculate_motor_omega(wheel_speeds);

  double shaft_torque = input_torque;

  // Passive drivetrain drag opposes shaft rotation in both forward and reverse.
  if (std::abs(motor_omega) > 1e-3) {
    double drag_sign = (motor_omega > 0.0) ? 1.0 : -1.0;
    double drag_torque = (p->viscous_drag_coeff * std::abs(motor_omega)) + p->coulomb_drag;
    shaft_torque -= drag_sign * drag_torque;
  }

  double axle_torque = shaft_torque * p->efficiency * p->gear_ratio;

  double delta_omega = (wheel_speeds.rear_left - wheel_speeds.rear_right) * p->gear_ratio;
  double avg_speed =
      ((std::abs(wheel_speeds.rear_left) + std::abs(wheel_speeds.rear_right)) / 2.0) *
      p->gear_ratio;
  double smoothing = std::clamp(avg_speed / 0.5, 0.0, 1.0);
  double delta_tau = p->kv * delta_omega * smoothing;
  delta_tau = std::clamp(delta_tau, -p->t_max, p->t_max);

  common_lib::structures::Wheels torques;
  torques.front_left = 0.0;
  torques.front_right = 0.0;
  torques.rear_left = (axle_torque / 2.0) - delta_tau;
  torques.rear_right = (axle_torque / 2.0) + delta_tau;

  return torques;
}
