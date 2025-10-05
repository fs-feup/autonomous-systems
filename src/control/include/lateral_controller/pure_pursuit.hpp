#pragma once

#include <memory>
#include <algorithm>

#include "base_lateral_controller.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/filters/low_pass_filter.hpp"
#include "control/include/config/parameters.hpp"
#include "utils/utils.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Pure Pursuit class
 *
 * @details
 * This class implements a Pure Pursuit controller.
 * This assumes a bicycle model for the vehicle.
 * The two main functions are:
 * - Calculate the lookahead point
 * - Calculate the steering angle (Pure Pursuit Control Law)
 */

class PurePursuit : public LateralController{
private:
  // Low pass filter to smooth the steering command
  std::shared_ptr<Filter> lpf_;

  // Last messages received from path planning, velocity estimation and SLAM
  std::vector<custom_interfaces::msg::PathPoint> last_path_msg_;
  custom_interfaces::msg::Velocities last_velocity_msg_;
  custom_interfaces::msg::Pose last_pose_msg_;
  double absolute_velocity_ = 0.0;

  // Whether the first message has been received for each type of message
  bool received_first_path_ = false;
  bool received_first_state_ = false;
  bool received_first_pose_ = false;

public:
  /**
   * @brief Construct a new Pure Pursuit object
   * @param params Control parameters
   */
  PurePursuit(const ControlParameters& params);

  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;

  void vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) override;

  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;

  double get_steering_command() override;

  /**
   * @brief Pure Pursuit control law
   *
   * @param rear_axis
   * @param cg center of gravity
   * @param lookahead_point
   * @param dist_cg_2_rear_axis distance between center of gravity and rear axis
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