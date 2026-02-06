#pragma once
#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct MotorParameters {
  float max_rpm;                 // Maximum rotational speed (RPM)
  float max_continuous_current;  // Continuous current limit (A)
  float max_peak_current;        // Peak current limit (A)
  float peak_duration;           // Time allowed at peak power (s)

  // Efficiency map (2D: RPM x Torque -> Efficiency)
  // Outer key = RPM, inner key = Torque, value = Efficiency
  std::map<float, std::map<float, float>> efficiency_map;

  std::map<float, float> torque_speed_continuous;  // RPM -> Max continuous torque (Nm)
  std::map<float, float> torque_speed_peak;        // RPM -> Max peak torque (Nm)

  MotorParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters