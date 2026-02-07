#pragma once

#include "base_tire_model.hpp"

class PacejkaCombinedSlip : public TireModel {
public:
  explicit PacejkaCombinedSlip(const common_lib::car_parameters::CarParameters& car_parameters)
      : TireModel(car_parameters){};
      
  std::pair<double, double> tire_forces(double slip_angle, double slip_ratio,
                                        double vertical_load) const override;
};