#include "pid/pid.hpp"

#include "gtest/gtest.h"

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output is saturated (limMax)
 */
TEST(PidTests, TestAntiWindUp1) {
  float antiWindupConst = 0.7;
  PID pid(0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, antiWindupConst);
  pid.proportional = 0.3;
  pid.integrator = 0.7;
  pid.differentiator = 0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(0.7 * antiWindupConst, pid.integrator);
}

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output is saturated (limMin)
 */
TEST(PidTests, TestAntiWindUp2) {
  float antiWindupConst = 0.7;
  PID pid(0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, antiWindupConst);
  pid.proportional = -0.3;
  pid.integrator = -0.7;
  pid.differentiator = -0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(-0.7 * antiWindupConst, pid.integrator);
}

/**
 * @brief Test PID class - AntiWindUp
 * Anti windup when output isnt saturated
 */
TEST(PidTests, TestAntiWindUp3) {
  float antiWindupConst = 0.7;
  PID pid(0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, antiWindupConst);
  pid.proportional = 0.3;
  pid.integrator = 0.3;
  pid.differentiator = 0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(0.3, pid.integrator);
}

/**
 * @brief Test PID class - calculate_proportional_term
 */
TEST(PidTests, ProportionalTerm) {
  float error = 4;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.calculate_proportional_term(error);
  EXPECT_FLOAT_EQ(1.6, pid.proportional);
}

/**
 * @brief Test PID class - calculate_integral_term
 * error positive
 */
TEST(PidTests, IntegralTerm1) {
  float error = 3;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.integrator = 0.3;
  pid.prevError = 4;
  pid.calculate_integral_term(error);
  EXPECT_FLOAT_EQ(0.405, pid.integrator);
}

/**
 * @brief Test PID class - calculate_integral_term
 * error negative
 */
TEST(PidTests, IntegralTerm2) {
  float error = -3;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.integrator = 0.3;
  pid.prevError = -4;
  pid.calculate_integral_term(error);
  EXPECT_FLOAT_EQ(0.195, pid.integrator);
}

/**
 * @brief Test PID class - calculate_derivative_term
 * measurement positive
 */
TEST(PidTests, DerivativeTerm1) {
  float measurement = 3;
  PID pid(0.4, 0.3, 0.1, 0.7, 0.1, -1, 1, 0.45);
  pid.differentiator = 0.4;
  pid.prevMeasurement = 4;
  pid.calculate_derivative_term(measurement);
  EXPECT_FLOAT_EQ(0.48, pid.differentiator);
}

/**
 * @brief Test PID class - calculate_derivative_term
 * measurement negative
 */
TEST(PidTests, DerivativeTerm2) {
  float measurement = -4;
  PID pid(0.4, 0.3, 0.1, 0.7, 0.1, -1, 1, 0.45);
  pid.differentiator = 0.4;
  pid.prevMeasurement = -1.2;
  pid.calculate_derivative_term(measurement);
  EXPECT_FLOAT_EQ(0.72, pid.differentiator);
}

/**
 * @brief Test PID class - compute_output
 * output not saturated
 */
TEST(PidTests, Output1) {
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
  pid.proportional = 0.3;
  pid.integrator = 0.1;
  pid.differentiator = 0.4;
  pid.compute_output();
  EXPECT_FLOAT_EQ(0.8, pid.out);
}

/**
 * @brief Test PID class - compute_output
 * output saturated LimMax
 */
TEST(PidTests, Output2) {
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
  pid.proportional = 0.3;
  pid.integrator = 0.3;
  pid.differentiator = 0.6;
  pid.compute_output();
  EXPECT_FLOAT_EQ(1, pid.out);
}

/**
 * @brief Test PID class - compute_output
 * output saturated LimMin
 */
TEST(PidTests, Output3) {
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
  pid.proportional = -0.3;
  pid.integrator = -0.3;
  pid.differentiator = -0.6;
  pid.compute_output();
  EXPECT_FLOAT_EQ(-1, pid.out);
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
  pid.integrator = 0.2;
  pid.differentiator = 0.1;
  pid.prevError = 1.5;
  pid.prevMeasurement = 3.5;
  pid.update(setpoint, measurement);
  EXPECT_NEAR(0.924, pid.out, 0.001);
}