#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct BatteryParameters {
  // Battery parameters
  int cells_series;
  int cells_parallel;
  float capacity_ah;
  float initial_soc;
  float max_discharge_current;
  float max_charge_current;
  float min_voltage;
  float min_soc;

  // Open circuit voltage
  float OCV_a5;
  float OCV_a4;
  float OCV_a3;
  float OCV_a2;
  float OCV_a1;
  float OCV_a0;

  // Ohmic resistance
  float R0_a5;
  float R0_a4;
  float R0_a3;
  float R0_a2;
  float R0_a1;
  float R0_a0;

  // Polarization resistance
  float R1_a5;
  float R1_a4;
  float R1_a3;
  float R1_a2;
  float R1_a1;
  float R1_a0;

  // Polarization capacitance
  float C1_a5;
  float C1_a4;
  float C1_a3;
  float C1_a2;
  float C1_a1;
  float C1_a0;

  BatteryParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters