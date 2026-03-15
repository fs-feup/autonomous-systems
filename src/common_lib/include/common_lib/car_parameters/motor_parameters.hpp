#pragma once
#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct MotorParameters {
  double max_peak_rpm;            // Maximum rotational peak speed (RPM)
  double max_continuous_rpm;      // Maximum continuous rotational speed (RPM)
  double max_peak_current;        // Maximum peak current (A)
  double max_continuous_current;  // Maximum continuous current (A)
  double max_continuous_power;    // Maximum continuous power (W)
  double max_peak_power;          // Maximum peak power (W)
  double max_continous_torque;    // Maximum continuous torque (Nm)
  double max_peak_torque;         // Maximum peak torque (Nm)
  double kt_constant;             // Torque constant (Nm/A)
  double peak_duration;           // Time allowed at peak power (s)

  // Efficiency map (2D: RPM x Torque -> Efficiency)
  // Outer key = RPM, inner key = Torque, value = Efficiency
  std::map<double, std::map<double, double>> efficiency_map;

  MotorParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters