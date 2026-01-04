#pragma once

/**
 * @brief Configuration parameters for the Velocity Planning class.
 *
 */
struct VelocityPlanningConfig {
  /**
   * @brief Minimum speed the car will consider on the velocity planning lookahead
   *
   */
  double minimum_velocity_ = 3;

  /**
   * @brief The desired velocity of the car without any velocity planning
   *
   */
  double desired_velocity_ = 5;

  /**
   * @brief Maximum braking acceleration
   *
   */
  double braking_acceleration_ = -4;
  /**
   * @brief Maximum foward acceleration
   *
   */
  double acceleration_ = 7.0;
  /**
   * @brief The maximum normal acceleration
   *
   */
  double normal_acceleration_ = 7;
  /**
   * @brief Flag to enable/disable velocity planning
   *
   */
  bool use_velocity_planning_ = true;

  VelocityPlanningConfig() = default;
};