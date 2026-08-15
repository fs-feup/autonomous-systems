#pragma once

#include <map>

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct AeroParameters {
  double lift_coefficient;
  double drag_coefficient;
  double aero_side_force_coefficient;
  double aero_balance_front;
  double frontal_area;
  double air_density;
  double ride_height_front;  // Ride height used to select coefficients from the map (mm)
  double ride_height_rear;   // Ride height used to select coefficients from the map (mm)

  // Aero coefficient maps (2D: Ride Height Front x Ride Height Rear -> coefficient)
  // Outer key = RHF (mm), inner key = RHR (mm)
  std::map<double, std::map<double, double>> cd_map;
  std::map<double, std::map<double, double>> cl_map;

  AeroParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters
