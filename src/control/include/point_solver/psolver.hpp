#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/structures/structures.hpp"
#include "common_lib/vehicle_dynamics/car_parameters.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

class PointSolver {
private:
  double k_;         /**< Lookahead gain */
  double ld_margin_; /**< Lookahead distance margin, a percentange of ld_ */
  /**< Distance from the center of gravity to the rear axis */

public:
  double dist_cg_2_rear_axis_ = DIST_CG_2_REAR_AXIS;
  Pose vehicle_pose_; /**< Vehicle pose */
  /**
   * @brief PointSolver Constructor
   */
  PointSolver(double k, double ld_margin);

  /**
   * @brief Find the closest point on the path
   *
   * @param path
   * @param rear_axis_point
   *
   * @return std::pair<Point, int> closest point and index
   */
  std::pair<Point, int> update_closest_point(
      const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
      Point rear_axis_point) const;

  /**
   * @brief Update Lookahead point
   *
   * @param path
   * @return std::pair<Point, double, bool> lookahead point, velocity and error status (1 = error)
   */
  std::tuple<Point, double, bool> update_lookahead_point(
      const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array, Point rear_axis_point,
      int closest_point_id) const;

  /**
   * @brief Calculate rear axis coordinates
   *
   * @param cg
   * @param heading
   * @param dist_cg_2_rear_axis
   *
   * @return Point
   */
  Point cg_2_rear_axis(Point cg, double heading, double dist_cg_2_rear_axis) const;

  /**
   * @brief update the LookaheadDistance based on a new velocity
   */
  double update_lookahead_distance(double k, double velocity) const;

  /**
   * @brief Update vehicle pose
   *
   * @param pose msg
   */
  void update_vehicle_pose(const custom_interfaces::msg::VehicleState &vehicle_state_msg);

  FRIEND_TEST(PointSolverTests, Test_update_closest_point_1);
  FRIEND_TEST(PointSolverTests, Test_update_lookahead_point_1);
};