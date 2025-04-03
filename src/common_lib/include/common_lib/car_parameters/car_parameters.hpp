#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

namespace common_lib::car_parameters {
struct CarParameters {
  double wheel_diameter = 0.406;
  double wheelbase = 1.530;
  double rear_axis_to_camera = 0.79;
  double axis_length = 1.2;
  double dist_cg_2_rear_axis = 0.804;
  double gear_ratio = 4;
  CarParameters();
  CarParameters(double wheel_diameter, double wheelbase, double rear_axis_to_camera,
                double axis_length, double dist_cg_2_rear_axis, double gear_ratio);
};

}  // namespace common_lib::car_parameters