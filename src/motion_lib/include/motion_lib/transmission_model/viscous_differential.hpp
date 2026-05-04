#pragma once

#include "base_transmission_model.hpp"

/**
 * @brief Single-speed rear-drive transmission with viscous LSD behavior.
 */
class ViscousDifferential : public TransmissionModel {
public:
  ViscousDifferential(const common_lib::car_parameters::CarParameters& car_parameters)
      : TransmissionModel(car_parameters) {}

  double calculate_motor_omega(const common_lib::structures::Wheels& wheel_speeds) const override;

  common_lib::structures::Wheels calculate_wheel_torques(
      double input_torque, const common_lib::structures::Wheels& wheel_speeds) const override;
};
