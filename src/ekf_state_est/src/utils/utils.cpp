#include "utils/utils.hpp"

std::pair<double, double> wheels_velocities_to_cg(std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters,
    double rl_rpm, [[maybe_unused]] double fl_rpm, double rr_rpm, [[maybe_unused]] double fr_rpm,
    double steering_angle) {
  std::pair<double, double> velocities = {0, 0};
  double rl_velocity =
      common_lib::maths::get_wheel_velocity_from_rpm(rl_rpm, car_parameters->wheel_diameter);
  double rr_velocity =
      common_lib::maths::get_wheel_velocity_from_rpm(rr_rpm, car_parameters->wheel_diameter);
  if (steering_angle == 0) {  // If no steering angle, moving straight
    velocities.first = (rl_velocity + rr_velocity) / 2;
  } else if (steering_angle > 0) {
    double rear_axis_center_rotation_radius = car_parameters->wheelbase / std::tan(steering_angle);
    velocities.second =
        rl_velocity / (rear_axis_center_rotation_radius - (car_parameters->track_width / 2));
    velocities.first = std::sqrt(std::pow(rear_axis_center_rotation_radius, 2) +
                            std::pow(car_parameters->dist_cg_2_rear_axis, 2)) *
                       std::fabs(velocities.second);
  } else {
    double rear_axis_center_rotation_radius = car_parameters->wheelbase / std::tan(steering_angle);
    velocities.second =
        rr_velocity / (rear_axis_center_rotation_radius + (car_parameters->track_width / 2));
    velocities.first = std::sqrt(std::pow(rear_axis_center_rotation_radius, 2) +
                            std::pow(car_parameters->dist_cg_2_rear_axis, 2)) *
                       std::fabs(velocities.second);
  }
  return velocities;
}
