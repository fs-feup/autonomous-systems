#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct DifferentialParameters {
  // Still no params

  DifferentialParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters