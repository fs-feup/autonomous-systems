#pragma once

#include "base_longitudinal_controller.hpp"
#include "common_lib/structures/position.hpp"
#include "utils/utils.hpp"
#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>

/**
 * @brief PI-D Controller class
 *
 * @details
 * This class implements a PI-D controller.
 * Its a PI-D because the P and I terms operates on the Error signal and the D works with the
 * feedback signal (measurement).
 *
 */
class PID : public LongitudinalController {
private:
  std::vector<custom_interfaces::msg::PathPoint> last_path_msg_;
  custom_interfaces::msg::Velocities last_velocity_msg_;
  custom_interfaces::msg::Pose last_pose_msg_;
  double absolute_velocity_ = 0.0;

  bool received_first_path_ = false;
  bool received_first_state_ = false;
  bool received_first_pose_ = false;

  double proportional_{0.0f};   /**< Proportional term current value */
  double integrator_{0.0f};     /**< Integrator term current value */
  double differentiator_{0.0f}; /**< Differentiator term current value */

  double prev_error_{0.0f};       /**< Previous error value, required for integrator */
  double prev_measurement_{0.0f}; /**< Previous measurement value, required for defferentiator */

  double out_{0.0f}; /**< Current output value */

  /**
   * @brief Calculate the error signal
   *
   * @param setpoint
   * @param measurement
   * @return error (double)
   */
  double calculate_error(double setpoint, double measurement) const;

  /**
   * @brief Calculate the proportional term
   *
   * @param error
   */
  void calculate_proportional_term(double error);

  /**
   * @brief Calculate the integral term
   *
   * @param error
   */
  void calculate_integral_term(double error);

  /**
   * @brief Anti-wind-up via dynamic integrator clamping
   */
  void anti_wind_up();

  /**
   * @brief Calculate the derivative term (derivative on measurement)
   *
   * @param measurement
   */
  void calculate_derivative_term(double measurement);

  /**
   * @brief Compute the output value and apply limits
   */
  void compute_output();

  /**
   * @brief Calculate the output value
   *
   * @param setpoint goal value
   * @param measurement current value
   * @return double
   */
  double update(double setpoint, double measurement);

public:
  /**
   * @brief Construct a new PID object
   * @param params Control parameters
   */
  PID(const ControlParameters &params);

  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;
  void vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) override;
  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;
  common_lib::structures::ControlCommand get_throttle_command() override;

  FRIEND_TEST(PidTests, TestAntiWindUp1);
  FRIEND_TEST(PidTests, TestAntiWindUp2);
  FRIEND_TEST(PidTests, TestAntiWindUp3);
  FRIEND_TEST(PidTests, ProportionalTerm);
  FRIEND_TEST(PidTests, IntegralTerm1);
  FRIEND_TEST(PidTests, IntegralTerm2);
  FRIEND_TEST(PidTests, DerivativeTerm1);
  FRIEND_TEST(PidTests, DerivativeTerm2);
  FRIEND_TEST(PidTests, Output1);
  FRIEND_TEST(PidTests, Output2);
  FRIEND_TEST(PidTests, Output3);
  FRIEND_TEST(PidTests, Update1);
};