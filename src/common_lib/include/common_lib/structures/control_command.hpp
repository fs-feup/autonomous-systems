#pragma once

namespace common_lib::structures {

struct ControlCommand {
  double throttle_fl = 0.0;  // Throttle command [-1.0, 1.0]
  double throttle_fr = 0.0;  // Throttle command [-1.0, 1.0]
  double throttle_rl = 0.0;  // Throttle command [-1.0, 1.0]
  double throttle_rr = 0.0;  // Throttle command [-1.0, 1.0]
  double steering_angle = 0.0;  // Steering angle command in radians

  ControlCommand() = default;
  ControlCommand(double throttle_fl, double throttle_fr, double throttle_rl, double throttle_rr, double steering_angle)
      : steering_angle(steering_angle), throttle_fl(throttle_fl), throttle_fr(throttle_fr), throttle_rl(throttle_rl), throttle_rr(throttle_rr) {}
};

}  // namespace common_lib::structures