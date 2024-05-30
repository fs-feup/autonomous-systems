#include "pid/pid.hpp"

#include "gtest/gtest.h"

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output is saturated (limMax)
 */
TEST(PidTests, TestAntiWindUp1) {
  float antiWindupConst = 0.7;
  PID pid(0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, antiWindupConst);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.7;
  pid.differentiator_ = 0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(0.7 * antiWindupConst, pid.integrator_);
}

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output is saturated (limMin)
 */
TEST(PidTests, TestAntiWindUp2) {
  float antiWindupConst = 0.7;
  PID pid(0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, antiWindupConst);
  pid.proportional_ = -0.3;
  pid.integrator_ = -0.7;
  pid.differentiator_ = -0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(-0.7 * antiWindupConst, pid.integrator_);
}

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output isnt saturated
 */
TEST(PidTests, TestAntiWindUp3) {
  float antiWindupConst = 0.7;
  PID pid(0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, antiWindupConst);
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
  float error = 4;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.calculate_proportional_term(error);
  EXPECT_FLOAT_EQ(1.6, pid.proportional_);
}

/**
 * @brief Test PID class - calculate_integral_term
 * error positive
 */
TEST(PidTests, IntegralTerm1) {
  float error = 3;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.integrator_ = 0.3;
  pid.prev_error_ = 4;
  pid.calculate_integral_term(error);
  EXPECT_FLOAT_EQ(0.405, pid.integrator_);
}

/**
 * @brief Test PID class - calculate_integral_term
 * error negative
 */
TEST(PidTests, IntegralTerm2) {
  float error = -3;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.integrator_ = 0.3;
  pid.prev_error_ = -4;
  pid.calculate_integral_term(error);
  EXPECT_FLOAT_EQ(0.195, pid.integrator_);
}

/**
 * @brief Test PID class - calculate_derivative_term
 * measurement positive
 */
TEST(PidTests, DerivativeTerm1) {
  float measurement = 3;
  PID pid(0.4, 0.3, 0.1, 0.7, 0.1, -1, 1, 0.45);
  pid.differentiator_ = 0.4;
  pid.prev_measurement_ = 4;
  pid.calculate_derivative_term(measurement);
  EXPECT_FLOAT_EQ(0.48, pid.differentiator_);
}

/**
 * @brief Test PID class - calculate_derivative_term
 * measurement negative
 */
TEST(PidTests, DerivativeTerm2) {
  float measurement = -4;
  PID pid(0.4, 0.3, 0.1, 0.7, 0.1, -1, 1, 0.45);
  pid.differentiator_ = 0.4;
  pid.prev_measurement_ = -1.2;
  pid.calculate_derivative_term(measurement);
  EXPECT_FLOAT_EQ(0.72, pid.differentiator_);
}

/**
 * @brief Test PID class - compute_output
 * output not saturated
 */
TEST(PidTests, Output1) {
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
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
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
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
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
  pid.proportional_ = -0.3;
  pid.integrator_ = -0.3;
  pid.differentiator_ = -0.6;
  pid.compute_output();
  EXPECT_FLOAT_EQ(-1, pid.out_);
}

// -2*kd(Measur - oldMeasur)+((2*tua-T)*oldDiffere)/(2*tau+T) + kp*Error + oldIntegra +
// 0.5*ki*T*(Error+OldErro)

/**
 * @brief Test PID class - update
 * Test all the methods
 */
TEST(PidTests, Update1) {
  float measurement = 2;
  float setpoint = 3;
  PID pid(0.4, 0.3, 0.1, 0.7, 0.1, -1, 1, 0.45);
  pid.integrator_ = 0.2;
  pid.differentiator_ = 0.1;
  pid.prev_error_ = 1.5;
  pid.prev_measurement_ = 3.5;
  pid.update(setpoint, measurement);
  EXPECT_NEAR(0.924, pid.out_, 0.001);
}