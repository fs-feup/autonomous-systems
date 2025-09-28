#pragma once

#include "base_aero_model.hpp"

class DefaultAeroModel : public AeroModel {
public:
  Eigen::Vector3d aero_forces(const Eigen::Vector3d& velocity) const override;
};