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
  pid.antiWindUp();
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
  pid.antiWindUp();
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
  pid.antiWindUp();
  EXPECT_FLOAT_EQ(0.3, pid.integrator);
}

/**
 * @brief Test PID class - calculateProportionalTerm
 */
TEST(PidTests, ProportionalTerm) {
  float error = 4;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.calculateProportionalTerm(error);
  EXPECT_FLOAT_EQ(1.6, pid.proportional);
}

/**
 * @brief Test PID class - calculateIntegralTerm
 * error positive
 */
TEST(PidTests, IntegralTerm1) {
  float error = 3;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.integrator = 0.3;
  pid.prevError = 4;
  pid.calculateIntegralTerm(error);
  EXPECT_FLOAT_EQ(0.405, pid.integrator);
}

/**
 * @brief Test PID class - calculateIntegralTerm
 * error negative
 */
TEST(PidTests, IntegralTerm2) {
  float error = -3;
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.5);
  pid.integrator = 0.3;
  pid.prevError = -4;
  pid.calculateIntegralTerm(error);
  EXPECT_FLOAT_EQ(0.195, pid.integrator);
}

/**
 * @brief Test PID class - calculateDerivativeTerm
 * measurement positive
 */
TEST(PidTests, DerivativeTerm1) {
  float measurement = 3;
  PID pid(0.4, 0.3, 0.1, 0.7, 0.1, -1, 1, 0.45);
  pid.differentiator = 0.4;
  pid.prevMeasurement = 4;
  pid.calculateDerivativeTerm(measurement);
  EXPECT_FLOAT_EQ(0.48, pid.differentiator);
}

/**
 * @brief Test PID class - calculateDerivativeTerm
 * measurement negative
 */
TEST(PidTests, DerivativeTerm2) {
  float measurement = -4;
  PID pid(0.4, 0.3, 0.1, 0.7, 0.1, -1, 1, 0.45);
  pid.differentiator = 0.4;
  pid.prevMeasurement = -1.2;
  pid.calculateDerivativeTerm(measurement);
  EXPECT_FLOAT_EQ(0.72, pid.differentiator);
}

/**
 * @brief Test PID class - computeOutput
 * output not saturated
 */
TEST(PidTests, Output1) {
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
  pid.proportional = 0.3;
  pid.integrator = 0.1;
  pid.differentiator = 0.4;
  pid.computeOutput();
  EXPECT_FLOAT_EQ(0.8, pid.out);
}

/**
 * @brief Test PID class - computeOutput
 * output saturated LimMax
 */
TEST(PidTests, Output2) {
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
  pid.proportional = 0.3;
  pid.integrator = 0.3;
  pid.differentiator = 0.6;
  pid.computeOutput();
  EXPECT_FLOAT_EQ(1, pid.out);
}

/**
 * @brief Test PID class - computeOutput
 * output saturated LimMin
 */
TEST(PidTests, Output3) {
  PID pid(0.4, 0.3, 0.09, 0.7, 0.1, -1, 1, 0.45);
  pid.proportional = -0.3;
  pid.integrator = -0.3;
  pid.differentiator = -0.6;
  pid.computeOutput();
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