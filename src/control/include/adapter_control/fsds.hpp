#pragma once

#include "adapter_control/adapter.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"

class Control;

class FsdsAdapter : public Adapter {
 private:
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr go_signal_sub_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr finished_signal_pub_;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_pub_;

 public:
  explicit FsdsAdapter(Control* control);

  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg);
  virtual void finish() override;
  virtual void publish_cmd(float acceleration = 0, float steering = 0) override;
};
