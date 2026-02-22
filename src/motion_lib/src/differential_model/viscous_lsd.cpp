#include "motion_lib/differential_model/viscous_lsd.hpp"

// Viscous limited slip differential model implementation
common_lib::structures::Wheels ViscousLSD::calculateTorqueDistribution(
    float input_torque, const common_lib::structures::Wheels& wheel_speeds) const {
  float delta_omega = wheel_speeds.rear_left - wheel_speeds.rear_right;
  float delta_tau = car_parameters_->differential_parameters->kv * delta_omega;

  if (delta_tau > car_parameters_->differential_parameters->t_max) {
    delta_tau = car_parameters_->differential_parameters->t_max;
  } else if (delta_tau < -car_parameters_->differential_parameters->t_max) {
    delta_tau = -car_parameters_->differential_parameters->t_max;
  }

  common_lib::structures::Wheels torques;
  torques.front_left = 0.0f;
  torques.front_right = 0.0f;
  torques.rear_left =
      (input_torque * car_parameters_->differential_parameters->efficiency / 2.0f) + delta_tau;
  torques.rear_right =
      (input_torque * car_parameters_->differential_parameters->efficiency / 2.0f) - delta_tau;

  return torques;
}