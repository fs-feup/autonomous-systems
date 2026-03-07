#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct BatteryParameters {
  // Battery parameters
  int cells_series;
  int cells_parallel;
  double capacity_ah;
  double initial_soc;
  double max_discharge_current;
  double max_charge_current;
  double min_voltage;
  double min_soc;

  // Open circuit voltage
  double OCV_a5;
  double OCV_a4;
  double OCV_a3;
  double OCV_a2;
  double OCV_a1;
  double OCV_a0;

  // Ohmic resistance
  double R0_a5;
  double R0_a4;
  double R0_a3;
  double R0_a2;
  double R0_a1;
  double R0_a0;

  // Polarization resistance
  double R1_a5;
  double R1_a4;
  double R1_a3;
  double R1_a2;
  double R1_a1;
  double R1_a0;

  // Polarization capacitance
  double C1_a5;
  double C1_a4;
  double C1_a3;
  double C1_a2;
  double C1_a1;
  double C1_a0;

  BatteryParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters