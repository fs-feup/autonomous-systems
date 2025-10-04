#pragma once

#include <memory>

#include "common_lib/structures/position.hpp"
#include "common_lib/filters/low_pass_filter.hpp"
#include "control/include/config/parameters.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

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
private:
  std::shared_ptr<ControlParameters> params_;
  std::shared_ptr<Filter> lpf_;

public:
  /**
   * @brief Construct a new Pure Pursuit object
   * @param params Control parameters
   */
  PurePursuit(const ControlParameters& params);

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

  double pp_steering_control_law(common_lib::structures::Position rear_axis,
                                 common_lib::structures::Position cg,
                                 common_lib::structures::Position lookahead_point,
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
  double calculate_alpha(common_lib::structures::Position vehicle_rear_wheel,
                         common_lib::structures::Position vehicle_cg,
                         common_lib::structures::Position lookahead_point, double rear_wheel_2_c_g);

  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_1);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_2);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_3);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_4);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_5);
  FRIEND_TEST(PurePursuitTests, Test_pp_steering_control_law_1);
};