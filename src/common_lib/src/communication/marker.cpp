#include "common_lib/communication/marker.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace common_lib::communication {

std::vector<common_lib::structures::PathPoint> path_point_array_from_ci_vector(
    const custom_interfaces::msg::PathPointArray &path_point_array) {
  std::vector<common_lib::structures::PathPoint> output;
  for (auto &p : path_point_array.pathpoint_array) {
    common_lib::structures::PathPoint point;
    point.position.x = p.x;
    point.position.y = p.y;
    point.ideal_velocity = p.v;
    output.push_back(point);
  }
  return output;
}

} // namespace common_lib::communication
