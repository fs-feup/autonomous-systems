#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct SteeringMotorParameters {
  double kp;
  double ki;
  double kd;
  double time_constant;

  SteeringMotorParameters(const std::string& config_name, const std::string& base_dir = "car");
};

}  // namespace common_lib::car_parameters