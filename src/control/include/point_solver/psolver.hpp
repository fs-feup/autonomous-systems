#ifndef POINT_SOLVER_HPP_
#define POINT_SOLVER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/structures/structures.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

/**< Distance from the center of gravity to the rear axis in m */
constexpr double DIST_CG_2_REAR_AXIS = 0.9822932352409;

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
      const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg, Point rear_axis_point);

  /**
   * @brief Update Lookahead point
   *
   * @param path
   * @return std::pair<Point, double, bool> lookahead point, velocity and error status (1 = error)
   */
  std::tuple<Point, double, bool> update_lookahead_point(
      const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg, Point rear_axis_point,
      int closest_point_id, double ld, double ld_margin);

  /**
   * @brief Calculate rear axis coordinates
   *
   * @param cg
   * @param heading
   * @param dist_cg_2_rear_axis
   *
   * @return Point
   */
  Point cg_2_rear_axis(Point cg, double heading, double dist_cg_2_rear_axis);

  /**
   * @brief update the LookaheadDistance based on a new velocity
   */
  double update_lookahead_distance(double k, double velocity);

  /**
   * @brief Update vehicle pose
   *
   * @param pose msg
   */
  void update_vehicle_pose(const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg);
};
#endif  // POINT_SOLVER_HPP_