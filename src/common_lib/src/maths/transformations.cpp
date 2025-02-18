#include "common_lib/maths/transformations.hpp"

#include <cmath>

namespace common_lib::maths {

double normalize_angle(double angle) {
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  while (angle >= M_PI) {
    angle -= 2 * M_PI;
  }
  return angle;
}

double get_wheel_velocity_from_rpm(const double rpm, const double wheel_diameter) {
  return rpm * wheel_diameter * M_PI / 60;
}

Eigen::Matrix2d get_rotation_matrix(const double angle) {
  Eigen::Matrix2d rotation_matrix;
  rotation_matrix << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle);
  return rotation_matrix;
}
}  // namespace common_lib::maths