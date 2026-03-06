#include "motion_lib/differential_model/viscous_lsd.hpp"

// Viscous limited slip differential model implementation
common_lib::structures::Wheels ViscousLSD::calculateTorqueDistribution(
    float input_torque, const common_lib::structures::Wheels& wheel_speeds) const {
  float delta_omega = wheel_speeds.rear_left - wheel_speeds.rear_right;
  float avg_speed = (std::abs(wheel_speeds.rear_left) + std::abs(wheel_speeds.rear_right)) / 2.0f;
  float smoothing = std::clamp(avg_speed / 0.5f, 0.0f, 1.0f);
  float delta_tau = car_parameters_->differential_parameters->kv * delta_omega * smoothing;

  float t_max = car_parameters_->differential_parameters->t_max;
  delta_tau = std::clamp(delta_tau, -t_max, t_max);

  double eff_torque = input_torque * car_parameters_->differential_parameters->efficiency *
                      car_parameters_->gear_ratio;

  common_lib::structures::Wheels torques;
  torques.front_left = 0.0f;
  torques.front_right = 0.0f;
  torques.rear_left = (eff_torque / 2.0f) - delta_tau;
  torques.rear_right = (eff_torque / 2.0f) + delta_tau;

  return torques;
}