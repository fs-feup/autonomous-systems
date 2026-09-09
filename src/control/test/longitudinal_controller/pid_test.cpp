#include "longitudinal_controller/pid.hpp"

#include <chrono>

#include "gtest/gtest.h"

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output is saturated (limMax)
 */
TEST(PidTests, TestAntiWindUp1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.7;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.7;
  pid.differentiator_ = 0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(0.7 * params.pid_anti_windup_, pid.integrator_);
}

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output is saturated (limMin)
 */
TEST(PidTests, TestAntiWindUp2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.7;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  pid.proportional_ = -0.3;
  pid.integrator_ = -0.7;
  pid.differentiator_ = -0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(-0.7 * params.pid_anti_windup_, pid.integrator_);
}

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output isnt saturated
 */
TEST(PidTests, TestAntiWindUp3) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.7;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.3;
  pid.differentiator_ = 0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(0.3, pid.integrator_);
}

/**
 * @brief Test PID class - calculate_proportional_term
 */
TEST(PidTests, ProportionalTerm) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.5;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  float error = 4;
  pid.calculate_proportional_term(error);
  EXPECT_FLOAT_EQ(1.6, pid.proportional_);
}

/**
 * @brief Test PID class - calculate_integral_term
 * error positive
 */
TEST(PidTests, IntegralTerm1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.5;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  float error = 3;
  pid.integrator_ = 0.3;
  pid.prev_error_ = 4;
  pid.calculate_integral_term(error, 0.1);
  EXPECT_FLOAT_EQ(0.39, pid.integrator_);
}

/**
 * @brief Test PID class - calculate_integral_term
 * error negative
 */
TEST(PidTests, IntegralTerm2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.5;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  float error = -3;
  pid.integrator_ = 0.3;
  pid.prev_error_ = -4;
  pid.calculate_integral_term(error, 0.1);
  EXPECT_FLOAT_EQ(0.21, pid.integrator_);
}

/**
 * @brief Test PID class - calculate_derivative_term
 * measurement positive
 */
TEST(PidTests, DerivativeTerm1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  float measurement = 3;
  pid.differentiator_ = 0.4;
  pid.prev_measurement_ = 4;
  pid.calculate_derivative_term(measurement, 0.1);
  EXPECT_FLOAT_EQ(1.0, pid.differentiator_);
}

/**
 * @brief Test PID class - calculate_derivative_term
 * measurement negative
 */
TEST(PidTests, DerivativeTerm2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  float measurement = -4;
  pid.differentiator_ = 0.4;
  pid.prev_measurement_ = -1.2;
  pid.calculate_derivative_term(measurement, 0.1);
  EXPECT_FLOAT_EQ(2.8, pid.differentiator_);
}

/**
 * @brief Test PID class - compute_output
 * output not saturated
 */
TEST(PidTests, Output1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.1;
  pid.differentiator_ = 0.4;
  pid.compute_output();
  EXPECT_FLOAT_EQ(0.8, pid.out_);
}

/**
 * @brief Test PID class - compute_output
 * output saturated LimMax
 */
TEST(PidTests, Output2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.3;
  pid.differentiator_ = 0.6;
  pid.compute_output();
  EXPECT_FLOAT_EQ(1, pid.out_);
}

/**
 * @brief Test PID class - compute_output
 * output saturated LimMin
 */
TEST(PidTests, Output3) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  pid.proportional_ = -0.3;
  pid.integrator_ = -0.3;
  pid.differentiator_ = -0.6;
  pid.compute_output();
  EXPECT_FLOAT_EQ(-1, pid.out_);
}

TEST(PidTests, UsesElapsedTimeForIntegrationAndDerivative) {
  ControlParameters params;
  params.pid_kp_ = 0.0;
  params.pid_ki_ = 1.0;
  params.pid_kd_ = 1.0;
  params.pid_lim_min_ = -1000;
  params.pid_lim_max_ = 1000;
  params.pid_anti_windup_ = 0.0;
  params.pid_max_positive_error_ = 1000;
  params.pid_max_negative_error_ = -1000;
  PID pid(params);

  pid.prev_error_ = 0.0;
  pid.prev_measurement_ = 0.0;
  pid.integrator_ = 0.0;
  pid.differentiator_ = 0.0;
  pid.last_update_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(20);
  pid.has_last_update_time_ = true;

  pid.update(1.0, 0.5);

  EXPECT_NEAR(0.02, pid.integrator_, 1e-6);
  EXPECT_NEAR(-25.0, pid.differentiator_, 1e-6);
}

/**
 * @brief Test PID class - update
 * Test all the methods
 */
TEST(PidTests, Update1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  params.pid_max_positive_error_ = 3;
  params.pid_max_negative_error_ = -3;
  PID pid(params);
  float measurement = 2;
  float setpoint = 3;
  pid.integrator_ = 0.2;
  pid.differentiator_ = 0.1;
  pid.prev_error_ = 1.5;
  pid.prev_measurement_ = 3.5;
  pid.update(setpoint, measurement);
  EXPECT_NEAR(1.0, pid.out_, 0.001);
}