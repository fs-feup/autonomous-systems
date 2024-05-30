#pragma once

#include "common_lib/structures/structures.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

/**< Maximum steering angle in rad */
constexpr double MAX_STEERING_ANGLE = 1.570;

/**< Minimum steering angle in rad */
constexpr double MIN_STEERING_ANGLE = -1.570;

/**< Wheel base of the vehicle in m */
constexpr double WHEEL_BASE = 1.5;

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
  double max_steering_angle_{MAX_STEERING_ANGLE}; /**< Maximum steering angle */
  double min_steering_angle_{MIN_STEERING_ANGLE}; /**< Minimum steering angle */
  double wheel_base_{WHEEL_BASE};                 /**< Wheel base of the vehicle */

  /**
   * @brief Construct a new Pure Pursuit object
   */
  PurePursuit();

  /**
   * @brief Pure Pursuit control law
   *
   * @param rear_axis
   * @param cg
   * @param lookahead_point
   * @param dist_cg_2_rear_axis
   *
   * @return Steering angle
   */

  double pp_steering_control_law(Point rear_axis, Point cg, Point lookahead_point,
                                 double dist_cg_2_rear_axis);

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
   * @brief Calculate the cross product of two vectors
   *
   * @param P1 - Origin of the vector
   * @param P2 - End of the vector
   * @param P3 - Point to calculate the cross product
   *
   * @return double
   */
  double cross_product(Point P1, Point P2, Point P3) const;

  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_1);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_2);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_3);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_4);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_5);
  FRIEND_TEST(PurePursuitTests, Test_pp_steering_control_law_1);
};