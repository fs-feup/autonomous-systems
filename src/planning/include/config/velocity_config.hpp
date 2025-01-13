#ifndef SRC_PLANNING_INCLUDE_CONFIG_VELOCITY_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_VELOCITY_CONFIG_HPP_

/**
 * @brief struct for the configuration of the velocity planning algorithm
 *
 */
struct VelocityPlanningConfig {
  /**
   * @brief minimum speed the car will consider on the velocity planning lookahead
   *
   */
  double minimum_velocity_ = 3;
  /**
   * @brief maximum braking acceleration
   *
   */
  double braking_acceleration_ = -4;
  /**
   * @brief the maximum normal acceleration
   *
   */
  double normal_acceleration_ = 7;
  /**
   * @brief flag to enable/disable velocity planning
   *
   */
  bool use_velocity_planning_ = true;
  VelocityPlanningConfig() = default;
  VelocityPlanningConfig(double minimum_velocity, double braking_acc, double normal_acc, bool use_velocity_planning)
      : minimum_velocity_(minimum_velocity),
      braking_acceleration_(braking_acc),
      normal_acceleration_(normal_acc),
      use_velocity_planning_(use_velocity_planning) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_VELOCITY_CONFIG_HPP_