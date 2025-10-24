#include "motion_lib/steering_model/parallel_front_steering.hpp"

Eigen::Vector4d ParallelFrontSteering::calculate_steering_angles(
    double steering_wheel_angle) const {
  return Eigen::Vector4d(steering_wheel_angle, steering_wheel_angle, 0, 0);
}