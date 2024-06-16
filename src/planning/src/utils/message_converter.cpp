#include "utils/message_converter.hpp"

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
    common_lib::structures::Cone new_cone = common_lib::structures::Cone((float)cone.position.x, (float)cone.position.y);
    cone_array.push_back(new_cone);
  }
  return cone_array;
}

visualization_msgs::msg::MarkerArray marker_array_from_path_point_array(
    const std::vector<custom_interfaces::msg::PathPoint> &path_point_array, std::string name_space,
    std::string frame_id, std::string color, std::string shape, float scale, int action) {
  visualization_msgs::msg::MarkerArray marker_array;
  std::array<float, 4> color_array = common_lib::communication::marker_color_map.at(color);

  for (size_t i = 0; i < path_point_array.size(); ++i) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = name_space;
    marker.id = i;
    marker.type = common_lib::communication::marker_shape_map.at(shape);
    marker.action = action;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.pose.position.x = path_point_array[i].x;
    marker.pose.position.y = path_point_array[i].y;
    marker.pose.position.z = 0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = color_array[0];
    marker.color.g = color_array[1];
    marker.color.b = color_array[2];
    marker.color.a = color_array[3];

    marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(5));

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}