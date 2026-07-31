#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct IndependentDriveParameters {
  double gear_ratio;                // Final drive ratio between motor and wheel speed
  double efficiency;                // Mechanical efficiency from motor shaft to wheels
  double viscous_drag_coeff;        // Shaft viscous drag coefficient [N.m / (rad/s)]
  double coulomb_drag;              // Shaft Coulomb drag [N.m]
  double coulomb_smooth_stiffness;  // Stiffness for atan smoothing in Coulomb drag

  IndependentDriveParameters(const std::string& config_name);
};

}  // namespace common_lib::car_parameters
