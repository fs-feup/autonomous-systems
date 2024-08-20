#include "common_lib/communication/marker.hpp"

namespace common_lib::communication {

visualization_msgs::msg::Marker marker_from_position(
    const common_lib::structures::Position& position, const std::string& name_space, const int id,
    const std::string& color, float scale, const std::string& frame_id, const std::string& shape,
    int action) {
  std::array<float, 4> color_array = marker_color_map.at(color);

  auto marker = visualization_msgs::msg::Marker();

  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = name_space;
  marker.id = id;
  marker.type = marker_shape_map.at(shape);
  marker.action = action;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.pose.position.x = position.x;
  marker.pose.position.y = position.y;
  marker.pose.position.z = 0.1;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = color_array[0];
  marker.color.g = color_array[1];
  marker.color.b = color_array[2];
  marker.color.a = color_array[3];

  return marker;
}

}  // namespace common_lib::communication