#pragma once

namespace common_lib::car_parameters {
struct CarParameters {
  double wheel_diameter = 0.5;
  double wheelbase = 1.530;
  double rear_axis_to_camera = 0.79;
  double axis_length = 1.2;
  double dist_cg_2_rear_axis = 0.9822932352409;
  double gear_ratio = 4;
};

}  // namespace common_lib::car_parameters