#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"

/**
 * @brief Base class for transmission models
 */
class TransmissionModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  TransmissionModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Returns equivalent motor shaft speed from wheel speeds.
   */
  virtual double calculate_motor_omega(
      const common_lib::structures::Wheels& wheel_speeds) const = 0;

  /**
   * @brief Applies transmission losses and distributes torque to wheels.
   */
  virtual common_lib::structures::Wheels calculate_wheel_torques(
      double input_torque, const common_lib::structures::Wheels& wheel_speeds) const = 0;
};
