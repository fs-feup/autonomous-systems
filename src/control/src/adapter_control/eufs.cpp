#include "adapter_control/eufs.hpp"

#include "node_/node_control.hpp"

EufsAdapter::EufsAdapter(Control* control)
    : Adapter(control),
      control_pub_(node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10)) {
  // No topic for eufs, just set the go_signal to true
  node_->go_signal_ = true;
}

void EufsAdapter::publish_cmd(double torque, double steering) {
  auto control_msg = ackermann_msgs::msg::AckermannDriveStamped();

  // Maybe normalize values if needed??
  control_msg.drive.acceleration = torque;
  control_msg.drive.steering_angle = steering;
  // control_msg.drive.jerk and control_msg.drive.steering_angle_velocity
  // should maybe be filled as well based on PID

  this->control_pub_->publish(control_msg);
}

