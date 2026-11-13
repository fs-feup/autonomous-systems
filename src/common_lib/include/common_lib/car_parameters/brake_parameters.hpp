#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct BrakeParameters {
  double max_front_torque;
  double max_rear_torque;

  BrakeParameters(const std::string& config_name, const std::string& base_dir = "car");
};

}  // namespace common_lib::car_parameters
