#pragma once

#include "base_brake_model.hpp"

class DefaultBrake : public BrakeModel {
public:
  explicit DefaultBrake(const common_lib::car_parameters::CarParameters& car_parameters)
      : BrakeModel(car_parameters) {}

  common_lib::structures::Wheels calculate_brake_torques(double brake_demand) const override;
};
