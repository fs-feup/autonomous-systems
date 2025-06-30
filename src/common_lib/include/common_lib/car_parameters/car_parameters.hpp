#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {
struct CarParameters {
  double wheel_diameter = 0.406;
  double wheelbase = 1.530;
  double rear_axis_to_camera = 0.79;
  double axis_length = 1.2;
  double dist_cg_2_rear_axis = 0.804;
  double gear_ratio = 4;
  double cog_height = 0.5;
  double lift_coefficient = -0.86;
  double drag_coefficient = 0.73;
  double aero_balance_front = 0.5;
  double frontal_area = 0.44;
  double mass = 145.0;
  double powertrainEfficiency = 0.95;
  double tire_B_lateral = 9.63;
  double tire_C_lateral = -1.39;
  double tire_D_lateral = 1.6;
  double tire_E_lateral = 1.0;
  double tire_B_longitudinal = 9.63;
  double tire_C_longitudinal = -1.39;
  double tire_D_longitudinal = 1.6;
  double tire_E_longitudinal = 1.0;
  double Izz = 101.082;
  CarParameters();
  CarParameters(double wheel_diameter, double wheelbase, double rear_axis_to_camera,
                double axis_length, double dist_cg_2_rear_axis, double gear_ratio);
};

}  // namespace common_lib::car_parameters