#pragma once

#include <memory>

#include "common_lib/structures/position.hpp"
#include "filters/lpf.hpp"
#include "gtest/gtest.h"
#include "lateral_controller/lateral_controller.hpp"
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

class PurePursuit : public LateralController {
private:
  /**
   * @brief Calculate alpha (angle between the vehicle and lookahead point)
   *
   * @param vehicle_rear_wheel
   * @param vehicle_cg
   * @param lookahead_point
   * @param dist_cg_2_rear_axis
   *
   * @return alpha angle in radians
   */
  double calculate_alpha(common_lib::structures::Position vehicle_rear_wheel,
                         common_lib::structures::Position vehicle_cg,
                         common_lib::structures::Position lookahead_point,
                         double dist_cg_2_rear_axis);

public:
  /**
   * @brief Construct a new Pure Pursuit object
   *
   * @param lpf Shared pointer to a low-pass filter
   * @param params Control parameters for the controller
   */
  PurePursuit(std::shared_ptr<Filter> lpf, const ControlParameters& params);

  /**
   * @brief Compute the steering control law using the Pure Pursuit method.
   * @details It calculates the steering angle based on the lookahead point and the vehicle's rear
   * axis. It is calculated using the Pure Pursuit formula.
   *
   * @param input Struct containing all necessary input data for the controller.
   * @return Steering angle in radians.
   */
  double steering_control_law(const LateralControlInput& input) override;

  FRIEND_TEST(PurePursuitTestFixture, Test_calculate_alpha_1);
  FRIEND_TEST(PurePursuitTestFixture, Test_calculate_alpha_2);
  FRIEND_TEST(PurePursuitTestFixture, Test_calculate_alpha_3);
  FRIEND_TEST(PurePursuitTestFixture, Test_calculate_alpha_4);
  FRIEND_TEST(PurePursuitTestFixture, Test_calculate_alpha_5);
  FRIEND_TEST(PurePursuitTestFixture, Test_pp_steering_control_law_1);
};