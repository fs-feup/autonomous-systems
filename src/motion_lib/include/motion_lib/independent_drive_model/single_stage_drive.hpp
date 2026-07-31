#pragma once

#include "base_independent_drive_model.hpp"

/**
 * @brief Single-stage drive model for independent motor-to-wheel drivetrains.
 */
class SingleStageDrive : public IndependentDriveModel {
public:
  SingleStageDrive(const common_lib::car_parameters::CarParameters& car_parameters)
      : IndependentDriveModel(car_parameters) {}

  double calculate_motor_omega(double wheel_speed) const override;

  double calculate_wheel_torque(double motor_torque, double wheel_speed) const override;
};
