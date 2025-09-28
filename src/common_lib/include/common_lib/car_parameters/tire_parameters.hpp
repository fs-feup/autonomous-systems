#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct TireParameters {
  double tire_B_lateral = 9.63;
  double tire_C_lateral = -1.39;
  double tire_D_lateral = 1.6;
  double tire_E_lateral = 1.0;
  double tire_B_longitudinal = 9.63;
  double tire_C_longitudinal = -1.39;
  double tire_D_longitudinal = 1.6;
  double tire_E_longitudinal = 1.0;
};

}  // namespace common_lib::car_parameters