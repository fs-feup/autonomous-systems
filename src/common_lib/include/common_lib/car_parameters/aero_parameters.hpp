#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct AeroParameters {
  double lift_coefficient = -0.86;
  double drag_coefficient = 0.73;
  double aero_side_force_coefficient = 0.0;
  double aero_balance_front = 0.5;
  double frontal_area = 0.44;
};

}  // namespace common_lib::car_parameters