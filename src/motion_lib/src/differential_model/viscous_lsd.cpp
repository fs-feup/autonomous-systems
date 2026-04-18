#include "motion_lib/differential_model/viscous_lsd.hpp"

// Viscous limited slip differential model implementation
common_lib::structures::Wheels ViscousLSD::calculateTorqueDistribution(
    double input_torque, const common_lib::structures::Wheels& wheel_speeds) const {
  double delta_omega = wheel_speeds.rear_left - wheel_speeds.rear_right;
  double avg_speed = (std::abs(wheel_speeds.rear_left) + std::abs(wheel_speeds.rear_right)) / 2.0;
  double smoothing = std::clamp(avg_speed / 0.5, 0.0, 1.0);
  double delta_tau = car_parameters_->differential_parameters->kv * delta_omega * smoothing;

  double t_max = car_parameters_->differential_parameters->t_max;
  delta_tau = std::clamp(delta_tau, -t_max, t_max);

  double eff_torque = input_torque * car_parameters_->differential_parameters->efficiency *
                      car_parameters_->gear_ratio;

  common_lib::structures::Wheels torques;
  torques.front_left = 0.0;
  torques.front_right = 0.0;
  torques.rear_left = (eff_torque / 2.0) - delta_tau;
  torques.rear_right = (eff_torque / 2.0) + delta_tau;

  return torques;
}