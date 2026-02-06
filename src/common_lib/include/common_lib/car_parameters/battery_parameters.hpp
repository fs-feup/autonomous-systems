#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct BatteryParameters {
  float nominal_voltage;                   // Nominal voltage (V)
  float capacity_ah;                       // Capacity in Amp-hours (Ah)
  float max_continuous_discharge_current;  // Max continuous discharge (A)
  float max_peak_discharge_current;        // Max peak discharge (A)
  float internal_resistance;               // Internal resistance (Ohm)
  float peak_duration;                     // Time allowed at peak current (s) - e.g., 30s
  std::map<float, float> soc_voltage_map;  // SOC -> Voltage (V)
  float min_voltage;                       // Minimum safe voltage (V)
  float max_voltage;                       // Maximum safe voltage (V)
  float min_soc;                           // Minimum safe SOC (0 to 1)

  BatteryParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters