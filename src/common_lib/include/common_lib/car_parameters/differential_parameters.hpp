#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct DifferentialParameters {
  double kv;          // Viscous coupling coefficient
  double t_max;       // Maximum torque transfer
  double efficiency;  // Efficiency of the differential

  DifferentialParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters