#pragma once

#include "base_tire_model.hpp"

class PacejkaCombinedSlip : public TireModel {
public:
  explicit PacejkaCombinedSlip(const common_lib::car_parameters::CarParameters& car_parameters)
      : TireModel(car_parameters){};
      
  Eigen::Vector3d tire_forces(const TireInput& tire_input)  override;
};