#include "remote_control/remote_control.hpp"

#include "gtest/gtest.h"

using remote_control::StopPid;
using remote_control::StopPidConfig;
using remote_control::make_control_command;
using remote_control::parse_emergency_control_message;

TEST(RemoteControlParsing, ParsesValidPayload) {
  const auto result = parse_emergency_control_message(
      R"({"type":"emergency_control","steering_rad":0.123,"throttle":-0.42,"e_ebs":false,"take_control":true,"sent_at_ms":123})");

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.123, result.payload.steering_rad);
  EXPECT_DOUBLE_EQ(-0.42, result.payload.throttle);
  EXPECT_FALSE(result.payload.e_ebs);
  EXPECT_TRUE(result.payload.take_control);
  EXPECT_EQ(123, result.payload.sent_at_ms);
}

TEST(RemoteControlParsing, RejectsMalformedPayload) {
  const auto result = parse_emergency_control_message(
      R"({"type":"emergency_control","steering_rad":0.0,"throttle":0.0,"e_ebs":false})");

  EXPECT_FALSE(result.valid);
}

TEST(RemoteControlParsing, ClampsControlsToAppLimits) {
  const auto result = parse_emergency_control_message(
      R"({"type":"emergency_control","steering_rad":1.0,"throttle":-3.0,"e_ebs":true,"take_control":true})");

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.335, result.payload.steering_rad);
  EXPECT_DOUBLE_EQ(-1.0, result.payload.throttle);
}

TEST(RemoteControlCommand, UsesRearThrottleAndSteeringRadians) {
  const auto command = make_control_command(0.12, -0.7);

  EXPECT_DOUBLE_EQ(0.0, command.throttle_fl);
  EXPECT_DOUBLE_EQ(0.0, command.throttle_fr);
  EXPECT_DOUBLE_EQ(-0.7, command.throttle_rl);
  EXPECT_DOUBLE_EQ(-0.7, command.throttle_rr);
  EXPECT_DOUBLE_EQ(0.12, command.steering);
}

TEST(RemoteControlEbsPid, PositiveMotorRpmCommandsNegativeThrottle) {
  StopPid pid(StopPidConfig{0.1, 0.0, 0.0, -1.0, 0.0, -1.0, 1.0});

  const double command = pid.update(-20.0, 0.025);

  EXPECT_DOUBLE_EQ(-1.0, command);
}

TEST(RemoteControlEbsPid, OutputIsLimitedToBrakeRange) {
  StopPid pid(StopPidConfig{0.1, 0.0, 0.0, -0.4, 0.0, -1.0, 1.0});

  const double command = pid.update(-20.0, 0.025);

  EXPECT_DOUBLE_EQ(-0.4, command);
}

TEST(RemoteControlEbsPid, DefaultScalingMapsTwoThousandRpmToPointFourControl) {
  StopPid pid(StopPidConfig{0.0002, 0.0, 0.0, -0.4, 0.4, -1.0, 1.0});

  const double brake_command = pid.update(-2000.0, 0.025);

  EXPECT_DOUBLE_EQ(-0.4, brake_command);
}
