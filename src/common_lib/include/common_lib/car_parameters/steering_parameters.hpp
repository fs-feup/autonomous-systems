#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct SteeringParameters {
  double minimum_steering_angle;
  double maximum_steering_angle;

  SteeringParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters