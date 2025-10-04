#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct SteeringParameters {
  double minimum_steering_angle = -0.335;
  double maximum_steering_angle = 0.335;
};

}  // namespace common_lib::car_parameters