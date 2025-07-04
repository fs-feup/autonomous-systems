#include "motion_lib/s2v_model/no_rear_wss_bicycle_model.hpp"

std::pair<double, double> NoRearWSSBicycleModel::wheels_velocities_to_cg(
    double rl_rpm, [[maybe_unused]] double fl_rpm, double rr_rpm, [[maybe_unused]] double fr_rpm,
    double steering_angle) {
  std::pair<double, double> velocities = {0, 0};
  double rl_velocity =
      common_lib::maths::get_wheel_velocity_from_rpm(rl_rpm, this->car_parameters_.wheel_diameter);
  double rr_velocity =
      common_lib::maths::get_wheel_velocity_from_rpm(rr_rpm, this->car_parameters_.wheel_diameter);
  if (steering_angle == 0) {  // If no steering angle, moving straight
    velocities.first = (rl_velocity + rr_velocity) / 2;
  } else if (steering_angle > 0) {
    double rear_axis_center_rotation_radius = this->car_parameters_.wheelbase / tan(steering_angle);
    velocities.second =
        rl_velocity / (rear_axis_center_rotation_radius - (this->car_parameters_.track_width / 2));
    velocities.first = sqrt(pow(rear_axis_center_rotation_radius, 2) +
                            pow(this->car_parameters_.dist_cg_2_rear_axis, 2)) *
                       fabs(velocities.second);
  } else {
    double rear_axis_center_rotation_radius = this->car_parameters_.wheelbase / tan(steering_angle);
    velocities.second =
        rr_velocity / (rear_axis_center_rotation_radius + (this->car_parameters_.track_width / 2));
    velocities.first = sqrt(pow(rear_axis_center_rotation_radius, 2) +
                            pow(this->car_parameters_.dist_cg_2_rear_axis, 2)) *
                       fabs(velocities.second);
  }
  return velocities;
}

common_lib::structures::Position NoRearWSSBicycleModel::rear_axis_position(
    const common_lib::structures::Position& cg, double orientation, double dist_cg_2_rear_axis) {
  common_lib::structures::Position rear_axis;
  rear_axis.x = cg.x - dist_cg_2_rear_axis * cos(orientation);
  rear_axis.y = cg.y - dist_cg_2_rear_axis * sin(orientation);
  return rear_axis;
}
