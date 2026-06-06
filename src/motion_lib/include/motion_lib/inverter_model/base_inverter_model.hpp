#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"

class InverterModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  explicit InverterModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  virtual ~InverterModel() = default;

  virtual double apply_throttle_curve(double throttle_input) const = 0;
};
