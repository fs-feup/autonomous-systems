#pragma once

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "adapter_control/adapter.hpp"

class Control;
class EufsAdapter : public Adapter {
 private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub;

 public:
  explicit EufsAdapter(Control* control);
  virtual void finish() override;
  virtual void publish_cmd(float acceleration = 0, float steering = 0) override;
};