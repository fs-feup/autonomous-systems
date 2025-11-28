#pragma once

#include <algorithm>
#include <memory>
#include <vector>     
#include <chrono>   

#include "base_lateral_controller.hpp"
#include "common_lib/filters/low_pass_filter.hpp"
#include "common_lib/structures/position.hpp"
#include "control/include/config/parameters.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "utils/utils.hpp"

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
class PurePursuit : public LateralController {
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


  // Distance loop states
  double previous_distance_error_ = 0.0;
  double integral_distance_error_ = 0.0;
  double distance_d_filt_ = 0.0;   // <-- filtered derivative state
  // Heading loop states
  double previous_heading_error_ = 0.0;
  double integral_heading_error_ = 0.0;
  double heading_d_filt_ = 0.0;    // <-- filtered derivative state

  // Anti-windup back-calculation gain (shared)
  double kaw_ = 1.0;               // tune ~ 0.5–2.0 (or ~1/max(Ki))

  // Timing
  std::chrono::steady_clock::time_point last_control_time_{};

public:
  /**
   * @brief Construct a new Pure Pursuit object
   * @param params Control parameters
   */
  PurePursuit(const ControlParameters& params);

  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;
  void vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) override;
  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;

  // Lateral PID that includes anti-windup back-calculation and derivative filtering.
  double lateral_pid(double distance_error, double heading_error, double dt, double u_pp,
                     double u_min, double u_max);

  double get_steering_command() override;

  // (Optional) legacy helper — keep only if you still call it elsewhere.
  double lateral_pid_error(double distance_error, double heading_error);

  /**
   * @brief Pure Pursuit control law
   *
   * @param rear_axis
   * @param cg center of gravity
   * @param lookahead_point
   * @param dist_cg_2_rear_axis distance between center of gravity and rear axis
   * @return Steering angle
   */
  double pp_steering_control_law(common_lib::structures::Position rear_axis,
                                 common_lib::structures::Position cg,
                                 common_lib::structures::Position lookahead_point,
                                 double dist_cg_2_rear_axis);

  /**
   * @brief Calculate alpha (angle between the vehicle and lookahead point)
   */
  double calculate_alpha(common_lib::structures::Position vehicle_rear_wheel,
                         common_lib::structures::Position vehicle_cg,
                         common_lib::structures::Position lookahead_point,
                         double rear_wheel_2_c_g);

  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_1);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_2);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_3);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_4);
  FRIEND_TEST(PurePursuitTests, Test_calculate_alpha_5);
  FRIEND_TEST(PurePursuitTests, Test_pp_steering_control_law_1);
};
