#include "common_lib/vehicle_dynamics/bicycle_model.hpp"

#include <cmath>
#include <iostream>
#include <utility>

#include "common_lib/maths/transformations.hpp"
#include "common_lib/vehicle_dynamics/car_parameters.hpp"

namespace common_lib::vehicle_dynamics {

std::pair<double, double> odometry_to_velocities_transform(double rl_rpm,
                                                           [[maybe_unused]] double fl_rpm,
                                                           double rr_rpm,
                                                           [[maybe_unused]] double fr_rpm,
                                                           double steering_angle) {
  std::pair<double, double> velocities = {0, 0};
  double rl_velocity = maths::get_wheel_velocity_from_rpm(rl_rpm, WHEEL_DIAMETER);
  double rr_velocity = maths::get_wheel_velocity_from_rpm(rr_rpm, WHEEL_DIAMETER);
  if (steering_angle == 0) {  // If no steering angle, moving straight
    velocities.first = (rl_velocity + rr_velocity) / 2;
  } else if (steering_angle > 0) {
    double rear_axis_center_rotation_radius = WHEELBASE / tan(steering_angle);
    velocities.second = rl_velocity / (rear_axis_center_rotation_radius - (AXIS_LENGTH / 2));
    velocities.first =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(REAR_AXIS_TO_CAMERA, 2)) *
        fabs(velocities.second);
  } else {
    double rear_axis_center_rotation_radius = WHEELBASE / tan(steering_angle);
    velocities.second = rr_velocity / (rear_axis_center_rotation_radius + (AXIS_LENGTH / 2));
    velocities.first =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(REAR_AXIS_TO_CAMERA, 2)) *
        fabs(velocities.second);
  }
  return velocities;
}

}  // namespace common_lib::vehicle_dynamics