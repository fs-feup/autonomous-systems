#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct SteeringMotorParameters {
  double kp = 0.00250;
  double ki = 0.01;
  double kd = 0.00001;
};

}  // namespace common_lib::car_parameters