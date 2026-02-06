#pragma once

#include "base_differential_model.hpp"
#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"

/**
 * @brief Limited slip differential model 
 */
class LimitedSlipDifferential : public DifferentialModel {
public:
  LimitedSlipDifferential(const common_lib::car_parameters::CarParameters& car_parameters)
      : DifferentialModel(car_parameters) {}

  common_lib::structures::Wheels calculateTorqueDistribution(
      float input_torque, const common_lib::structures::Wheels& wheel_speeds) const override;
};
