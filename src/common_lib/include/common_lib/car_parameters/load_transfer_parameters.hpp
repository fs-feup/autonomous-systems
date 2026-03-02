#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct LoadTransferParameters {
  double roll_axis_z;
  double front_roll_center_z;
  double rear_roll_center_z;
  double front_stiffness_distribution;
  double pitch_center_z;
  LoadTransferParameters(const std::string& config_path);
};
}  // namespace common_lib::car_parameters
