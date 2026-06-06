#pragma once

#include "base_inverter_model.hpp"

class LinearInverter : public InverterModel {
public:
  explicit LinearInverter(const common_lib::car_parameters::CarParameters& car_parameters)
      : InverterModel(car_parameters) {}

  double apply_throttle_curve(double throttle_input) const override;
};
