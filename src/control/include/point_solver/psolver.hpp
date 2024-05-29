#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/structures/structures.hpp"
#include "common_lib/vehicle_dynamics/car_parameters.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"

class PointSolver {
public:
  // double k_;                       /**< Lookahead gain */
  // double ld_;                      /**< Lookahead distance */
  // double ld_margin_;               /**< Lookahead distance margin, a percentange of ld_ */
  double dist_cg_2_rear_axis_ =
      DIST_CG_2_REAR_AXIS; /**< Distance from the center of gravity to the rear axis */
  Pose vehicle_pose_;      /**< Vehicle pose */
  // Point lookahead_point_;     /**< Lookahead point */
  // double lookahead_velocity_; /**< Lookahead velocity */
  // Point closest_point_;       /**< Closest point on the Path*/
  // int closest_point_id_ = -1; /**< Closest point on the Path*/

  /**
   * @brief PointSolver Constructer
   */
  PointSolver();

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
      int closest_point_id, double ld, double ld_margin) const;

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
};