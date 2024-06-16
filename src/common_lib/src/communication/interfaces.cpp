#include "common_lib/communication/interfaces.hpp"

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

custom_interfaces::msg::PathPointArray custom_interfaces_array_from_vector(
    const std::vector<common_lib::structures::PathPoint> &input_path) {
  auto message = custom_interfaces::msg::PathPointArray();
  for (auto const &element : input_path) {
    auto point = custom_interfaces::msg::PathPoint();
    point.x = element.position.x;
    point.y = element.position.y;
    point.v = element.ideal_velocity;
    message.pathpoint_array.push_back(point);
  }
  return message;
}

std::vector<common_lib::structures::Cone> cone_vector_from_custom_interfaces(
    const custom_interfaces::msg::ConeArray &msg) {
  std::vector<common_lib::structures::Cone> cone_array;
  for (const auto &cone : msg.cone_array) {
    if (cone.position.x == 0 && cone.position.y == 0) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cone at (0,0)");
    }
    common_lib::structures::Cone new_cone = common_lib::structures::Cone(static_cast<float>(cone.position.x), static_cast<float>(cone.position.y));
    cone_array.push_back(new_cone);
  }
  return cone_array;
}

}  // namespace common_lib::communication