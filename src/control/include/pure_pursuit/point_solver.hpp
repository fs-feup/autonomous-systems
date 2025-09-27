#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "config/parameters.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "gtest/gtest.h"
#include "motion_lib/car_parameters.hpp"
#include "motion_lib/s2v_model/bicycle_model.hpp"
#include "rclcpp/rclcpp.hpp"

class PointSolver {
private:
  std::shared_ptr<ControlParameters> params_;
  double k_;                   /**< Lookahead gain */
  double lookahead_minimum_;   /**< Minimum lookahead distance */
  double first_last_max_dist_; /**< Max dist between first and last point of the path to
                                  be considered a closed track*/
  BicycleModel bicycle_model_; /**< Bicycle model for vehicle dynamics */

public:
  double dist_cg_2_rear_axis_ =
      DIST_CG_2_REAR_AXIS; /**< Distance from the center of gravity to the rear axis */
  common_lib::structures::VehiclePose vehicle_pose_; /**< Vehicle pose */

  PointSolver(const ControlParameters &params);

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

  FRIEND_TEST(PointSolverTests, Test_update_closest_point_1);
  FRIEND_TEST(PointSolverTests, Test_update_lookahead_point_1);
};