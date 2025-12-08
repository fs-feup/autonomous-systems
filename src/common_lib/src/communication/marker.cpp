#include "common_lib/communication/marker.hpp"

namespace common_lib::communication {

visualization_msgs::msg::Marker marker_from_position(
    const common_lib::structures::Position& position, const std::string& name_space, const int id,
    const std::string& color, float scale, const std::string& frame_id, const std::string& shape,
    int action) {
  std::array<float, 4> color_array = marker_color_map().at(color);

  auto marker = visualization_msgs::msg::Marker();

  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = name_space;
  marker.id = id;
  marker.type = marker_shape_map().at(shape);
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

visualization_msgs::msg::Marker lines_marker_from_triangulations(
    const std::vector<std::pair<Point, Point>>& triangulations, const std::string& name_space,
    const std::string& frame_id, int id, const std::string& color, float scale, int action) {
  std::array<float, 4> color_array = marker_color_map().at(color);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = name_space;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = action;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;

  marker.color.r = color_array[0];
  marker.color.g = color_array[1];
  marker.color.b = color_array[2];
  marker.color.a = color_array[3];

  for (const auto& [point1, point2] : triangulations) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = point1.x();
    p1.y = point1.y();
    p1.z = 0.0;

    p2.x = point2.x();
    p2.y = point2.y();
    p2.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  return marker;
}

visualization_msgs::msg::MarkerArray velocity_hover_markers(
    const std::vector<common_lib::structures::PathPoint>& path_array, const std::string& name_space,
    const std::string& frame_id, float scale, int every_nth) {
  visualization_msgs::msg::MarkerArray marker_array;

  std::array<float, 4> color_array = {1.0f, 1.0f, 1.0f, 0.8f};

  for (size_t i = 0; i < path_array.size(); i += every_nth) {
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = frame_id;
    sphere_marker.header.stamp = rclcpp::Clock().now();
    sphere_marker.ns = name_space + "_sphere";
    sphere_marker.id = static_cast<int>(i);
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;

    sphere_marker.pose.position.x = path_array[i].position.x;
    sphere_marker.pose.position.y = path_array[i].position.y;
    sphere_marker.pose.position.z = 0.1;
    sphere_marker.pose.orientation.w = 1.0;

    sphere_marker.scale.x = scale;
    sphere_marker.scale.y = scale;
    sphere_marker.scale.z = scale;

    sphere_marker.color.r = color_array[0];
    sphere_marker.color.g = color_array[1];
    sphere_marker.color.b = color_array[2];
    sphere_marker.color.a = color_array[3];

    marker_array.markers.push_back(sphere_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = rclcpp::Clock().now();
    text_marker.ns = name_space + "_text";
    text_marker.id = static_cast<int>(i);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = path_array[i].position.x;
    text_marker.pose.position.y = path_array[i].position.y;
    text_marker.pose.position.z = 0.5;
    text_marker.pose.orientation.w = 1.0;

    std::ostringstream velocity_text;
    velocity_text << std::fixed << std::setprecision(1) << path_array[i].ideal_velocity << " m/s";

    text_marker.text = velocity_text.str();
    text_marker.scale.z = 0.3;

    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;

    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

}  // namespace common_lib::communication