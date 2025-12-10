#pragma once

#include "rclcpp/rclcpp.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"

namespace common_lib::communication {

/**
 * @brief Convert from custom interfaces PathPointArray to vector of common_lib PathPoints
 *
 * @param path_point_array
 * @return std::vector<common_lib::structures::PathPoint>
 */
std::vector<common_lib::structures::PathPoint> path_point_array_from_ci_vector(
    const custom_interfaces::msg::PathPointArray &path_point_array);

custom_interfaces::msg::ConeArray custom_interfaces_array_from_vector(
    const std::vector<common_lib::structures::Cone> &input_cones);

custom_interfaces::msg::PathPointArray custom_interfaces_array_from_vector(
    const std::vector<common_lib::structures::PathPoint> &input_path, bool is_map_closed);

std::vector<common_lib::structures::Cone> cone_vector_from_custom_interfaces(
    const custom_interfaces::msg::ConeArray &msg);


}  // namespace common_lib::communication