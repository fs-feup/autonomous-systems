#pragma once

#include "base_aero_model.hpp"

class DefaultAeroModel : public AeroModel {
public:
  DefaultAeroModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : AeroModel(car_parameters){}

  Eigen::Vector3d aero_forces(const Eigen::Vector3d& velocity) const override;
};