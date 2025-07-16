#include "pid/pid.hpp"

#include "gtest/gtest.h"
#include "node_/control_parameters.hpp"

TEST(PidTests, TestAntiWindUp1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.5;
  params.pid_t_ = 0.01;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.7;
  float antiWindupConst = params.pid_anti_windup_;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.7;
  pid.differentiator_ = 0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(0.7 * antiWindupConst, pid.integrator_);
}

TEST(PidTests, TestAntiWindUp2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.5;
  params.pid_t_ = 0.01;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.7;
  float antiWindupConst = params.pid_anti_windup_;
  PID pid(params);
  pid.proportional_ = -0.3;
  pid.integrator_ = -0.7;
  pid.differentiator_ = -0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(-0.7 * antiWindupConst, pid.integrator_);
}

TEST(PidTests, TestAntiWindUp3) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.5;
  params.pid_t_ = 0.01;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.7;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.3;
  pid.differentiator_ = 0.2;
  pid.anti_wind_up();
  EXPECT_FLOAT_EQ(0.3, pid.integrator_);
}

TEST(PidTests, ProportionalTerm) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.5;
  float error = 4;
  PID pid(params);
  pid.calculate_proportional_term(error);
  EXPECT_FLOAT_EQ(1.6, pid.proportional_);
}

TEST(PidTests, IntegralTerm1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.5;
  float error = 3;
  PID pid(params);
  pid.integrator_ = 0.3;
  pid.prev_error_ = 4;
  pid.calculate_integral_term(error);
  EXPECT_FLOAT_EQ(0.405, pid.integrator_);
}

TEST(PidTests, IntegralTerm2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.5;
  float error = -3;
  PID pid(params);
  pid.integrator_ = 0.3;
  pid.prev_error_ = -4;
  pid.calculate_integral_term(error);
  EXPECT_FLOAT_EQ(0.195, pid.integrator_);
}

TEST(PidTests, DerivativeTerm1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.1;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  float measurement = 3;
  PID pid(params);
  pid.differentiator_ = 0.4;
  pid.prev_measurement_ = 4;
  pid.calculate_derivative_term(measurement);
  EXPECT_FLOAT_EQ(0.48, pid.differentiator_);
}

TEST(PidTests, DerivativeTerm2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.1;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  float measurement = -4;
  PID pid(params);
  pid.differentiator_ = 0.4;
  pid.prev_measurement_ = -1.2;
  pid.calculate_derivative_term(measurement);
  EXPECT_FLOAT_EQ(0.72, pid.differentiator_);
}

TEST(PidTests, Output1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.1;
  pid.differentiator_ = 0.4;
  pid.compute_output();
  EXPECT_FLOAT_EQ(0.8, pid.out_);
}

TEST(PidTests, Output2) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  PID pid(params);
  pid.proportional_ = 0.3;
  pid.integrator_ = 0.3;
  pid.differentiator_ = 0.6;
  pid.compute_output();
  EXPECT_FLOAT_EQ(1, pid.out_);
}

TEST(PidTests, Output3) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.09;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  PID pid(params);
  pid.proportional_ = -0.3;
  pid.integrator_ = -0.3;
  pid.differentiator_ = -0.6;
  pid.compute_output();
  EXPECT_FLOAT_EQ(-1, pid.out_);
}

TEST(PidTests, Update1) {
  ControlParameters params;
  params.pid_kp_ = 0.4;
  params.pid_ki_ = 0.3;
  params.pid_kd_ = 0.1;
  params.pid_tau_ = 0.7;
  params.pid_t_ = 0.1;
  params.pid_lim_min_ = -1;
  params.pid_lim_max_ = 1;
  params.pid_anti_windup_ = 0.45;
  float measurement = 2;
  float setpoint = 3;
  PID pid(params);
  pid.integrator_ = 0.2;
  pid.differentiator_ = 0.1;
  pid.prev_error_ = 1.5;
  pid.prev_measurement_ = 3.5;
  pid.update(setpoint, measurement);
  EXPECT_NEAR(0.924, pid.out_, 0.001);
}