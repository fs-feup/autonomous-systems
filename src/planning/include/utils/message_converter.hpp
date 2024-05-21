#ifndef SRC_PLANNING_INCLUDE_COMMUNICATION_MESSAGE_CONVERTER_HPP
#define SRC_PLANNING_INCLUDE_COMMUNICATION_MESSAGE_CONVERTER_HPP

#include <chrono>
#include <vector>

#include "common_lib/communication/marker.hpp"
#include "common_lib/structures/path_point.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/cone.hpp"
#include "utils/pathpoint.hpp"

/**
 * @brief Convert from custom interfaces PathPointArray to vector of common_lib PathPoints
 *
 * @param path_point_array
 * @return std::vector<common_lib::structures::PathPoint>
 */
std::vector<common_lib::structures::PathPoint> path_point_array_from_ci_vector(
    const custom_interfaces::msg::PathPointArray &path_point_array);

std::vector<common_lib::structures::PathPoint> path_point_array_from_path_point_vector(
    const std::vector<PathPoint *> &path_point_array);

custom_interfaces::msg::PathPointArray custom_interfaces_array_from_vector(
    const std::vector<PathPoint *> &input_path);

custom_interfaces::msg::ConeArray custom_interfaces_array_from_vector(
    const std::vector<Cone *> &input_path);

std::vector<Cone *> cone_vector_from_custom_interfaces(
    const custom_interfaces::msg::ConeArray &msg);

template <typename T>
visualization_msgs::msg::MarkerArray marker_array_from_path_point_array(
    const std::vector<T *> &path_point_array, std::string name_space, std::string frame_id,
    std::string color = "red", std::string shape = "cylinder", float scale = 0.5,
    int action = visualization_msgs::msg::Marker::MODIFY) {
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

    marker.pose.position.x = path_point_array[i]->getX();
    marker.pose.position.y = path_point_array[i]->getY();
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

visualization_msgs::msg::MarkerArray marker_array_from_path_point_array(
    const std::vector<custom_interfaces::msg::PathPoint> &path_point_array, std::string name_space,
    std::string frame_id, std::string color = "red", std::string shape = "cylinder",
    float scale = 0.5, int action = visualization_msgs::msg::Marker::MODIFY);

#endif  // SRC_PLANNING_INCLUDE_COMMUNICATION_MESSAGE_CONVERTER_HPP