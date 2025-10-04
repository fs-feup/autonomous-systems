#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "control/include/config/parameters.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "gtest/gtest.h"
#include "utils/utils.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Find the closest point on the path
 *
 * @param path
 * @param rear_axis_point
 *
 * @return std::pair<Point, int> closest point and index
 */
std::tuple<common_lib::structures::Position, int, double> get_closest_point(
  const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array, const common_lib::structures::Position& position) ;

/**
 * @brief Update Lookahead point
 *
 * @param path
 * @return std::tuple<common_lib::structures::Position, double, bool> lookahead point, velocity
 * and error status (1 = error)
 */
std::tuple<common_lib::structures::Position, double, bool> get_lookahead_point(
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
    int closest_point_id, double lookahead_distance, common_lib::structures::Position rear_axis_position, double last_to_first_max_dist) ;

/**
 * @brief Calculate rear axis coordinates
 *
 * @param cg center of gravity position
 * @param orientation orientation of the vehicle in radians relative to the world frame (ccw)
 * @param dist_cg_2_rear_axis distance between the center of gravity and the rear axis
 *
 * @return Point
 */
common_lib::structures::Position rear_axis_position(
    const common_lib::structures::Position& cg, double orientation, double dist_cg_2_rear_axis);
