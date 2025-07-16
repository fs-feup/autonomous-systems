#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "gtest/gtest.h"
#include "motion_lib/car_parameters.hpp"
#include "motion_lib/s2v_model/bicycle_model.hpp"
#include "node_/control_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

class PointSolver {
private:
  double k_;                   /**< Lookahead gain */
  double lookahead_minimum_;   /**< Minimum lookahead distance */
  BicycleModel bicycle_model_; /**< Bicycle model for vehicle dynamics */
  ControlParameters params_;   /**< Control parameters */

public:
  common_lib::structures::VehiclePose vehicle_pose_; /**< Vehicle pose */

  /**
   * @brief PointSolver Constructor
   */
  explicit PointSolver(const ControlParameters &params);

  /**
   * @brief Find the closest point on the path
   *
   * @param path
   * @param rear_axis_point
   *
   * @return std::pair<Point, int> closest point and index
   */
  std::tuple<common_lib::structures::Position, int, double> update_closest_point(
      const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array) const;

  /**
   * @brief Update Lookahead point
   *
   * @param path
   * @return std::tuple<common_lib::structures::Position, double, bool> lookahead point, velocity
   * and error status (1 = error)
   */
  std::tuple<common_lib::structures::Position, double, bool> update_lookahead_point(
      const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
      int closest_point_id) const;

  /**
   * @brief update the LookaheadDistance based on a new velocity
   */
  double update_lookahead_distance(double k, double velocity) const;

  /**
   * @brief Update vehicle pose
   *
   * @param pose msg
   */
  void update_vehicle_pose(const custom_interfaces::msg::Pose &vehicle_state_msg,
                           double velocity = 0.0);

  /**
   * @brief Get the next point in the path after the closest point
   *
   * @param pathpoint_array
   * @param closest_point_id
   * @return Position of the next closest point in path
   */
  std::tuple<common_lib::structures::Position, bool> next_closest_point(
      const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
      int closest_point_id) const;

  FRIEND_TEST(PointSolverTests, Test_update_closest_point_1);
  FRIEND_TEST(PointSolverTests, Test_update_lookahead_point_1);
};