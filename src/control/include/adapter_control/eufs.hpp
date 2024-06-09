#pragma once

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "node_/node_control.hpp"

class EufsAdapter : public Control {
 private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub_;
  rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr vehicle_state_sub_;

 public:
  explicit EufsAdapter();
  void vehicle_state_callback(const eufs_msgs::msg::CarState& msg);
  void publish_cmd(double acceleration = 0, double steering = 0) override;
};