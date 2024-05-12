#include "common_lib/structures/structures.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

/**< Wheel base of the vehicle in m */
constexpr double WHEEL_BASE = 1.5;

/**< Distance from the center of gravity to the rear axis in m */
constexpr double DIST_CG_2_REAR_AXIS = 0.5;

/**
 * @brief Pure Pursuit class
 *
 * @details
 * This class implements a Pure Pursuit controller.
 * The two main functions are:
 * - Calculate the lookahead point
 * - Calculate the steering angle (Pure Pursuit Controll Law)
 */

class PurePursuit {
 public:
  double k_;                       /**< Lookahead gain */
  double ld_;                      /**< Lookahead distance */
  double ld_margin_;               /**< Lookahead distance margin, a percentange of ld_ */
  double wheel_base_ = WHEEL_BASE; /**< Wheel base of the vehicle */
  double dist_cg_2_rear_axis_ =
      DIST_CG_2_REAR_AXIS;    /**< Distance from the center of gravity to the rear axis */
  Pose vehicle_pose_;         /**< Vehicle pose */
  Point lookahead_point_;     /**< Lookahead point */
  Point closest_point_;       /**< Closest point on the Path*/
  int closest_point_id_ = -1; /**< Closest point on the Path*/

  /**
   * @brief Construct a new Pure Pursuit object
   *
   * @param k Lookahead gain
   * @param ld_margin Lookahead distance margin, a percentange of ld_
   */
  PurePursuit(double k, double ld_margin);

  /**
   * @brief Update Lookahead point
   *
   * @param path
   * @return std::pair<Point, int> lookahead point and error status (1 = error)
   */
  std::pair<Point, bool> update_lookahead_point(
      const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg);

  /**
   * @brief Find the closest point on the path
   *
   * @param path
   * @return std::pair<Point, int> closest point and index
   */
  std::pair<Point, int> update_closest_point(
      const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg);

  /**
   * @brief Calculate steering angle
   *
   */
  double update_steering_angle(
      const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg,
      const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg);

  /**
   * @brief update the LookaheadDistance based on a new velocity
   */
  double update_lookahead_distance();

  /**
   * @brief Update vehicle pose
   *
   * @param pose msg
   */
  void update_vehicle_pose(const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg);

  /**
   * @brief Pure Pursuit control law
   *
   * @return Steering angle
   */

  double pp_steering_control_law();

  /**
   * @brief Calculate alpha (angle between the vehicle and lookahead point)
   *
   * @param vehicle_rear_wheel
   * @param vehicle_cg
   * @param lookahead_point
   * @param dist_cg_2_rear_axis
   *
   * @return double
   */
  double calculate_alpha(Point vehicle_rear_wheel, Point vehicle_cg, Point lookahead_point,
                         double rear_wheel_2_c_g);

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
};