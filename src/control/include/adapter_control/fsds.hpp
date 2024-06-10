#pragma once

#include "node_/node_control.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"


class FsdsAdapter : public Control {
 private:
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr go_signal_sub_;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_pub_;

 public:
  explicit FsdsAdapter(const ControlParameters &params);

  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg);
  void publish_cmd(double acceleration = 0, double steering = 0) override;
};
