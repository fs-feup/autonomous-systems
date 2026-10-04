#pragma once

#include <yaml-cpp/yaml.h>

#include <map>
#include <string>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct InverterParameters {
  double efficiency;
  double max_phase_current;

  // Torque setpoint ramp times, for the full command range.
  double acceleration_ramp_ms;
  double regen_braking_ramp_ms;

  // Regen torque limit as a fraction of the drive limit.
  double regen_torque_fraction = 1.0;

  // Torque limit programmed into the inverter, by mission, and the one in use.
  // The throttle command is a fraction of max_torque; the motor's own limits
  // apply on top of it and are independent of the mode.
  std::map<std::string, double> modes;
  std::string selected_mode;
  double max_torque = 0.0;

  InverterParameters(const std::string& config_name);
};

}  // namespace common_lib::car_parameters
