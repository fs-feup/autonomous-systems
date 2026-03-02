#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"

/**
 * @brief Base class for differential models
 */
class DifferentialModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  DifferentialModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  /**
   * @brief Calculate torque distribution to each wheel
   *
   * @param input_torque Total torque from the motor (Nm)
   * @return Torque distribution to each wheel (front left, front right, rear left, rear right) in
   * Nm
   */
  virtual common_lib::structures::Wheels calculateTorqueDistribution(
      float input_torque, const common_lib::structures::Wheels& wheel_speeds) const = 0;
};
