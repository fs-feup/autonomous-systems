#pragma once

#include "base_tire_model.hpp"

class PacejkaBombado : public TireModel {
public:
  std::pair<double, double> tire_forces(double slip_angle, double slip_ratio,
                                        double vertical_load) const override;
};