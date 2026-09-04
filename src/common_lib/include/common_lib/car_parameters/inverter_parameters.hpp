#pragma once

#include <yaml-cpp/yaml.h>

#include <map>
#include <string>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct InverterParameters {
  double efficiency;
  double max_phase_current;
  double acceleration_delay_ms;
  double coast_delay_ms;
  double regen_braking_delay_ms;

  InverterParameters(const std::string& config_name);
};

}  // namespace common_lib::car_parameters
