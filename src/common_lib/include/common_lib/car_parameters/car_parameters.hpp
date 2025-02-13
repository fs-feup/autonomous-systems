#pragma once

namespace common_lib::car_parameters {
struct CarParameters {
  double wheel_diameter = 0.5;
  double wheelbase = 1.530;
  double rear_axis_to_camera = 0.79;
  double axis_length = 1.2;
  double dist_cg_2_rear_axis = 0.9822932352409;
  double gear_ratio = 4;
  CarParameters() = default;
  CarParameters(double wheel_diameter, double wheelbase, double rear_axis_to_camera,
                double axis_length, double dist_cg_2_rear_axis, double gear_ratio)
      : wheel_diameter(wheel_diameter),
        wheelbase(wheelbase),
        rear_axis_to_camera(rear_axis_to_camera),
        axis_length(axis_length),
        dist_cg_2_rear_axis(dist_cg_2_rear_axis),
        gear_ratio(gear_ratio) {}
};

}  // namespace common_lib::car_parameters