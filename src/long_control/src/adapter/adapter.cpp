#include "adapter/adapter.hpp"
#include "node_/node_long_control.hpp"

Adapter::Adapter(std::string mode, LongitudinalControl *long_control) {
  this->node = long_control;
  this->mode = mode;

  RCLCPP_INFO(this->node->get_logger(), "mode: %s", mode.c_str());

  if (mode == "fsds") {
    this->fsds_init();
  }
}

void Adapter::fsds_init() {
  this->fsds_state_subscription_ = this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&Adapter::fsds_mission_state_callback, this, std::placeholders::_1));
  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
  this->fsds_cmd_publisher_ =
      this->node->create_publisher<fs_msgs::msg::ControlCommand>("/control_command", 10);
}

void Adapter::fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  // Mission is unnecessary
  // TODO: just info that the message is being received
  return;
}

void Adapter::fsds_publish_cmd(float throttle, float brake){
  // Throttle [0, 1] - Steering [-1, 1] - Brake [0, 1]
  auto message = fs_msgs::msg::ControlCommand();

  // TODO: Convert values to range according to the units sent by AS

  message.throttle = throttle;
  message.steering = 0;
  message.brake = brake;

  this->fsds_cmd_publisher_->publish(message);
}

void Adapter::publish_cmd(float throttle, float brake){
  if (this->mode == "fsds") {
    this->fsds_publish_cmd(throttle, brake);
  }
}

void Adapter::fsds_finish(){
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true; // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}