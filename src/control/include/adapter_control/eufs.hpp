#pragma once

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "adapter_control/adapter.hpp"

class EufsAdapter : public Adapter {
 private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub_;

 public:
  explicit EufsAdapter(Control* control);
  void publish_cmd(double acceleration = 0, double steering = 0) override;
};