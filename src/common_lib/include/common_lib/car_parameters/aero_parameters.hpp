#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct AeroParameters {
  double lift_coefficient;
  double drag_coefficient;
  double aero_side_force_coefficient;
  double aero_balance_front;
  double frontal_area;
  double air_density;

  AeroParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters