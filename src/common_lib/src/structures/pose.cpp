#include "common_lib/structures/pose.hpp"

namespace common_lib::structures {

Pose::Pose(Position position, double orientation, double orientation_noise, rclcpp::Time timestamp)
    : position(position),
      orientation(orientation),
      orientation_noise(orientation_noise),
      timestamp(timestamp) {}

Pose::Pose(double x, double y, double theta, double x_noise, double y_noise, double theta_noise,
           rclcpp::Time timestamp)
    : position(x, y, x_noise, y_noise, timestamp),
      orientation(theta),
      orientation_noise(theta_noise),
      timestamp(timestamp) {}

}  // namespace common_lib::structures