#include "communicators/eufs.hpp"

#include <stdio.h>
#include <unistd.h>

#include <functional>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "custom_interfaces/msg/odom.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

EufsCommunicator::EufsCommunicator(Orchestrator* orchestrator) : Communicator() {
  this->orchestrator_ = orchestrator;
  this->sim_publisher_ =
      this->orchestrator_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
  this->state_subscriber_ = this->orchestrator_->create_subscription<custom_interfaces::msg::State>(
      "/ros_can/state", 10,
      std::bind(&EufsCommunicator::send_state_from_car, this, std::placeholders::_1));
  this->odom_subscriber_ =
      this->orchestrator_->create_subscription<custom_interfaces::msg::Odom>(
          "/ros_can/wheel_speeds", 10,
          std::bind(&EufsCommunicator::send_odom_from_car, this, std::placeholders::_1));
}

void EufsCommunicator::send_to_car(const custom_interfaces::msg::VcuCommand msg) {
  auto command = ackermann_msgs::msg::AckermannDriveStamped();
  command.drive.steering_angle = msg.steering_angle_request;
  command.drive.speed = msg.axle_speed_request;
  this->sim_publisher_->publish(command);
}

void EufsCommunicator::send_state_from_car(const custom_interfaces::msg::State msg) {
  this->orchestrator_->publish_state(msg);
}

void EufsCommunicator::send_odom_from_car(const custom_interfaces::msg::Odom msg) {
  this->orchestrator_->publish_odom(msg);
}
