#include "src/long_control/include/adapter/fsds.hpp"

#include "src/long_control/include/node_/node_long_control.hpp"

FsdsAdapter::FsdsAdapter(LongitudinalControl* long_control) : Adapter(long_control) {
  this->init();
}

void FsdsAdapter::init() {
  this->fsds_state_subscription_ = this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&FsdsAdapter::fsds_mission_state_callback, this, std::placeholders::_1));
  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
  this->fsds_cmd_publisher_ =
      this->node->create_publisher<fs_msgs::msg::ControlCommand>("/control_command", 10);
}

void FsdsAdapter::fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  std::cout << "Set mission undefined for Ads Dv\n";
  return;
}

void FsdsAdapter::publish_cmd(float acceleration, float braking, float steering) {
  // Throttle [0, 1] - Steering [-1, 1] - Brake [0, 1]
  auto message = fs_msgs::msg::ControlCommand();

  // TODO(andre): Convert values to range according to the units sent by AS

  message.throttle = acceleration;
  message.brake = braking;
  message.steering = steering;

  this->fsds_cmd_publisher_->publish(message);
}

void FsdsAdapter::finish() {
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true;  // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}