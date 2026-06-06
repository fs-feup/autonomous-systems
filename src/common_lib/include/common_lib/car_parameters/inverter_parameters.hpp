#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct InverterParameters {
  InverterParameters(const std::string& config_name);
};

}  // namespace common_lib::car_parameters
