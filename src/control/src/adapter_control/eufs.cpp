#include "adapter_control/eufs.hpp"

#include "node_/node_control.hpp"

EufsAdapter::EufsAdapter(Control* control)
    : Adapter(control),
      control_pub(node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10)) {
  // No topic for eufs, just set the go_signal to true
  node->go_signal = true;
}

void EufsAdapter::publish_cmd(double acceleration, double steering) {
  auto control_msg = ackermann_msgs::msg::AckermannDriveStamped();

  // Maybe normalize values if needed??
  control_msg.drive.acceleration = acceleration;
  control_msg.drive.steering_angle = steering;
  // control_msg.drive.jerk and control_msg.drive.steering_angle_velocity
  // should maybe be filled as well based on PID

  this->control_pub->publish(control_msg);
}

