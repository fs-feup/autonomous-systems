#pragma once
#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct MotorParameters {
  float max_peak_rpm;            // Maximum rotational peak speed (RPM)
  float max_continuous_rpm;      // Maximum continuous rotational speed (RPM)
  float max_peak_current;        // Maximum peak current (A)
  float max_continuous_current;  // Maximum continuous current (A)
  float max_continuous_power;    // Maximum continuous power (W)
  float max_peak_power;          // Maximum peak power (W)
  float max_continous_torque;    // Maximum continuous torque (Nm)
  float max_peak_torque;         // Maximum peak torque (Nm)
  float kt_constant;             // Torque constant (Nm/A)
  float peak_duration;           // Time allowed at peak power (s)

  // Efficiency map (2D: RPM x Torque -> Efficiency)
  // Outer key = RPM, inner key = Torque, value = Efficiency
  std::map<float, std::map<float, float>> efficiency_map;

  MotorParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters