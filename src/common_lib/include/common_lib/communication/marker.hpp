#pragma once

#include <map>
#include <type_traits>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace common_lib::communication {

const std::map<std::string, std::array<float, 4>, std::less<>> marker_color_map = {
    {"blue", {0.0, 0.0, 1.0, 1.0}},
    {"yellow", {1.0, 1.0, 0.0, 1.0}},
    {"orange", {1.0, 0.5, 0.0, 1.0}},
    {"red", {1.0, 0.0, 0.0, 1.0}},
    {"green", {0.0, 1.0, 0.0, 1.0}}};

const std::map<std::string, int, std::less<>> marker_shape_map = {
    {"cylinder", visualization_msgs::msg::Marker::CYLINDER},
    {"cube", visualization_msgs::msg::Marker::CUBE},
    {"sphere", visualization_msgs::msg::Marker::SPHERE},
    {"line", visualization_msgs::msg::Marker::LINE_STRIP}};

/**
 * @brief A helper struct to check if a type T has a member named 'position'.
 *        This base template assumes that T does not have a 'position' member.
 *
 * @tparam T The type to check.
 * @tparam typename A helper typename defaulted to void, used for SFINAE.
 */
template <typename T, typename = void>
struct has_position : std::false_type {};

/**
 * @brief A specialization of the has_position struct for types T that do have a valid 'position'
 * member. This template is instantiated when T has a 'position' member with 'x' and 'y' which are
 * arithmethic sub-members (numbers).
 *
 * @tparam T The type to check.
 */
template <typename T>
struct has_position<T,
                    std::enable_if_t<std::is_arithmetic_v<decltype(std::declval<T>().position.x)> &&
                                     std::is_arithmetic_v<decltype(std::declval<T>().position.y)>>>
    : std::true_type {};

/**
 * @brief Converts a vector of cones to a marker array
 *
 * @param cone_array vector of cones
 * @param color color of the marker (blue, yellow, orange, red, green)
 * @param shape shape of the marker (cylinder, cube, sphere)
 * @param frame_id frame id of the marker, for transforms
 * @param name_space namespace of the marker, used in conjunction with ID to identify marker
 * @param scale scale of the marker, default is 0.5
 * @param action action of the marker, default is ADD/MODIFY
 * @return visualization_msgs::msg::MarkerArray
 */
template <typename T>
visualization_msgs::msg::MarkerArray marker_array_from_structure_array(
    const std::vector<T>& structure_array, const std::string& name_space,
    const std::string& frame_id, const std::string& color = "red",
    const std::string& shape = "cylinder", float scale = 0.5,
    int action = visualization_msgs::msg::Marker::MODIFY) {
  static_assert(
      has_position<T>::value,
      "Template argument T must have a data member named 'position' with 'x' and 'y' sub-members");

  visualization_msgs::msg::MarkerArray marker_array;
  std::array<float, 4> color_array = marker_color_map.at(color);

  for (size_t i = 0; i < structure_array.size(); ++i) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = name_space;
    marker.id = i;
    marker.type = marker_shape_map.at(shape);
    marker.action = action;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.pose.position.x = structure_array[i].position.x;
    marker.pose.position.y = structure_array[i].position.y;
    marker.pose.position.z = 0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = color_array[0];
    marker.color.g = color_array[1];
    marker.color.b = color_array[2];
    marker.color.a = color_array[3];

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

/**
 * @brief Converts a vector of cones to a marker array
 *
 * @param cone_array vector of cones
 * @param color color of the marker (blue, yellow, orange, red, green)
 * @param shape shape of the marker (cylinder, cube, sphere)
 * @param frame_id frame id of the marker, for transforms
 * @param name_space namespace of the marker, used in conjunction with ID to identify marker
 * @param scale scale of the marker, default is 0.5
 * @param action action of the marker, default is ADD/MODIFY
 * @return visualization_msgs::msg::MarkerArray
 */
template <typename T>
visualization_msgs::msg::Marker line_marker_from_structure_array(
    const std::vector<T>& structure_array, const std::string& name_space,
    const std::string& frame_id, const int id, const std::string& color = "red",
    const std::string& shape = "line", float scale = 0.1,
    int action = visualization_msgs::msg::Marker::MODIFY) {
  static_assert(
      has_position<T>::value,
      "Template argument T must have a data member named 'position' with 'x' and 'y' sub-members");

  std::array<float, 4> color_array = marker_color_map.at(color);

  visualization_msgs::msg::Marker marker;

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

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = color_array[0];
  marker.color.g = color_array[1];
  marker.color.b = color_array[2];
  marker.color.a = color_array[3];

  for (size_t i = 0; i < structure_array.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = structure_array[i].position.x;
    point.y = structure_array[i].position.y,

    marker.points.push_back(point);
  }

  return marker;
}


}  // namespace common_lib::communication