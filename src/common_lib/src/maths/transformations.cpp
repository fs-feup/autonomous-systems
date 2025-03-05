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

Eigen::VectorXd global_to_local_coordinates(const Eigen::Vector3d& local_reference_frame,
                                            const Eigen::VectorXd& global_points) {
  const double x = local_reference_frame(0);
  const double y = local_reference_frame(1);
  const double angle = local_reference_frame(2);
  const Eigen::Matrix2d rotation_matrix = get_rotation_matrix(-angle);
  Eigen::VectorXd local_points(global_points.size());
  for (int i = 0; i < global_points.size(); i += 2) {
    const double x_global = global_points(i);
    const double y_global = global_points(i + 1);
    const Eigen::Vector2d global_point(x_global - x, y_global - y);
    const Eigen::Vector2d local_point = rotation_matrix * global_point;
    local_points(i) = local_point(0);
    local_points(i + 1) = local_point(1);
  }
  return local_points;
}

Eigen::VectorXd local_to_global_coordinates(const Eigen::Vector3d& local_reference_frame,
                                            const Eigen::VectorXd& local_points) {
  const double x = local_reference_frame(0);
  const double y = local_reference_frame(1);
  const double angle = local_reference_frame(2);
  const Eigen::Matrix2d rotation_matrix = get_rotation_matrix(angle);
  Eigen::VectorXd global_points_vector(local_points.size());
  for (int i = 0; i < local_points.size(); i += 2) {
    const double x_local = local_points(i);
    const double y_local = local_points(i + 1);
    const Eigen::Vector2d local_point(x_local, y_local);
    const Eigen::Vector2d global_point = rotation_matrix * local_point;
    global_points_vector(i) = global_point(0) + x;
    global_points_vector(i + 1) = global_point(1) + y;
  }
  return global_points_vector;
}

}  // namespace common_lib::maths