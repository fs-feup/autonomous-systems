#include "motion_lib/independent_drive_model/single_stage_drive.hpp"

#include <algorithm>
#include <cmath>

double SingleStageDrive::calculate_motor_omega(double wheel_speed) const {
  return wheel_speed * car_parameters_->independent_drive_parameters->gear_ratio;
}

double SingleStageDrive::calculate_wheel_torque(double motor_torque, double wheel_speed) const {
  const auto& p = car_parameters_->independent_drive_parameters;

  double motor_omega = calculate_motor_omega(wheel_speed);

  double shaft_torque = motor_torque;

  double viscous_drag = p->viscous_drag_coeff * motor_omega;

  double smooth_sign_omega = (2.0 / M_PI) * std::atan(p->coulomb_smooth_stiffness * motor_omega);
  double coulomb_drag = p->coulomb_drag * smooth_sign_omega;

  shaft_torque -= (viscous_drag + coulomb_drag);

  return shaft_torque * p->efficiency * p->gear_ratio;
}
