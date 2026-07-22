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

  // Viscous Drag
  double viscous_drag = p->viscous_drag_coeff * motor_omega;

  // Coulomb Drag
  // Uses the atan activation function to smoothly flip direction around 0 rad/s
  double smooth_sign_omega = (2.0 / M_PI) * std::atan(p->coulomb_smooth_stiffness * motor_omega);
  double coulomb_drag = p->coulomb_drag * smooth_sign_omega;

  // Apply both resistances to the shaft torque
  shaft_torque -= (viscous_drag + coulomb_drag);

  double axle_torque = shaft_torque * p->efficiency * p->gear_ratio;

  double delta_omega = (wheel_speeds.rear_left - wheel_speeds.rear_right);
  double avg_speed = ((std::abs(wheel_speeds.rear_left) + std::abs(wheel_speeds.rear_right)) / 2.0);
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
