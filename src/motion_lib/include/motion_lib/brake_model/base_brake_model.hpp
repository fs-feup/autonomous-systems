#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"

class BrakeModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  explicit BrakeModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  virtual ~BrakeModel() = default;

  virtual common_lib::structures::Wheels calculate_brake_torques(double brake_demand) const = 0;
};
