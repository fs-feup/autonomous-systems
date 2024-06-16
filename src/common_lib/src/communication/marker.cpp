#include "common_lib/communication/marker.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace common_lib::communication {

visualization_msgs::msg::Marker arrow_marker_from_two_markers(
    const visualization_msgs::msg::Marker &m1, const visualization_msgs::msg::Marker &m2,
    std::string name_space, std::string frame_id, int id, std::string color, float scale,
    int action) {
  visualization_msgs::msg::Marker marker;
  std::array<float, 4> color_array = common_lib::communication::marker_color_map.at(color);
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = name_space;
  marker.id = id;
  marker.type = common_lib::communication::marker_shape_map.at("arrow");
  marker.action = action;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = scale * 0.1;
  marker.scale.y = scale * 0.1;
  marker.scale.z = scale * 0.1;

  marker.color.r = color_array[0];
  marker.color.g = color_array[1];
  marker.color.b = color_array[2];
  marker.color.a = color_array[3];
  geometry_msgs::msg::Point point1;
  point1.x = m1.pose.position.x;
  point1.y = m1.pose.position.y;
  point1.z = 0;
  marker.points.push_back(point1);
  geometry_msgs::msg::Point point2;
  point2.x = m2.pose.position.x;
  point2.y = m2.pose.position.y;
  point2.z = 0;
  marker.points.push_back(point2);
  return marker;
}

}  // namespace common_lib::communication