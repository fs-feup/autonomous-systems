#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

#include "aero_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "steering_motor_parameters.hpp"
#include "tire_parameters.hpp"

namespace common_lib::car_parameters {
struct CarParameters {
  double wheel_diameter = 0.406;
  double wheelbase = 1.530;
  double track_width = 1.2;
  double dist_cg_2_rear_axis = 0.804;
  double gear_ratio = 4;

  double cog_height = 0.5;
  double mass = 145.0;
  double powertrainEfficiency = 0.95;
  double Izz = 101.082;
  common_lib::car_parameters::TireParameters tire_parameters;
  common_lib::car_parameters::AeroParameters aero_parameters;
  common_lib::car_parameters::SteeringMotorParameters steering_motor_parameters;
  CarParameters();
  CarParameters(double wheel_diameter, double wheelbase, double track_width,
                double dist_cg_2_rear_axis, double gear_ratio);
};

}  // namespace common_lib::car_parameters