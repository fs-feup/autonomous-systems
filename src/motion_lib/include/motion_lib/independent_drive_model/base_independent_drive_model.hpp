#pragma once

#include <memory>

#include "common_lib/car_parameters/car_parameters.hpp"

/**
 * @brief Base class for independent drive models
 */
class IndependentDriveModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  IndependentDriveModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Returns equivalent motor shaft speed from a single wheel speed.
   */
  virtual double calculate_motor_omega(double wheel_speed) const = 0;

  /**
   * @brief Applies drivetrain losses and returns the wheel torque for a single wheel.
   */
  virtual double calculate_wheel_torque(double motor_torque, double wheel_speed) const = 0;
};
