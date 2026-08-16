#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct TransmissionParameters {
  double gear_ratio;                // Final drive ratio between motor and wheel speed
  double efficiency;                // Mechanical efficiency from motor shaft to wheels
  double kv;                        // Viscous coupling coefficient for LSD torque transfer
  double t_max;                     // Maximum LSD torque transfer
  double viscous_drag_coeff;        // Shaft viscous drag coefficient [N.m / (rad/s)]
  double coulomb_drag;              // Shaft Coulomb drag [N.m]
  double coulomb_smooth_stiffness;  // Stiffness for atan smoothing in Coulomb drag
  
  double preload;                   // Static locking torque (Nm) for Salisbury LSD
  double drive_ramp_effect;         // Drive ramp locking multiplier
  double coast_ramp_effect;         // Coast ramp locking multiplier

  TransmissionParameters(const std::string& config_name);
};

}  // namespace common_lib::car_parameters