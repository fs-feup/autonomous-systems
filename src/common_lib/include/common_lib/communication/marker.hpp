#pragma once

#include <map>

#include "common_lib/structures/cone.hpp"
#include "std_msgs/msg/color_rgba.hpp"
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
    {"sphere", visualization_msgs::msg::Marker::SPHERE}};

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
visualization_msgs::msg::MarkerArray marker_array_from_cone_array(
    std::vector<common_lib::structures::Cone> cone_array, std::string name_space,
    std::string frame_id, std::string color = "red", std::string shape = "cylinder",
    float scale = 0.5, int action = visualization_msgs::msg::Marker::MODIFY);
}  // namespace common_lib::communication