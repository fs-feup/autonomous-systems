#include "adapter_control/fsds.hpp"

#include "node_/node_control.hpp"

FsdsAdapter::FsdsAdapter(Control* control)
    : Adapter(control),
      go_signal_sub_(node_->create_subscription<fs_msgs::msg::GoSignal>(
          "/signal/go", 10,
          std::bind(&FsdsAdapter::fsds_mission_state_callback, this, std::placeholders::_1))),
      control_pub_(node_->create_publisher<fs_msgs::msg::ControlCommand>("/control_command", 10)) {}

void FsdsAdapter::fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  // TODO: I don't see any info relevant in GoSignal msg so I'm assuming it's a trigger
  // node->go_signal = true;
  return;
}

void FsdsAdapter::publish_cmd(double acceleration, double steering) {
  // Throttle [0, 1] - Steering [-1, 1] - Brake [0, 1]
  auto message = fs_msgs::msg::ControlCommand();

  // TODO(andre): Convert values to range according to the units sent by AS

  message.throttle = acceleration;
  message.steering = steering;

  this->control_pub_->publish(message);
}

