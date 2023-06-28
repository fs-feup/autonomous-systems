#include "communicators/eufs.hpp"

#include <stdio.h>
#include <unistd.h>

#include <cstdio>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

EufsCommunicator::EufsCommunicator(Orchestrator* orchestrator) {
  this->orchestrator_ = orchestrator;

  this->publisher_ =
      orchestrator_->create_publisher<ackermann_msgs::msg::ackermann_drive_stamped>("/cmd", 10);
  this->subscriber_ = orchestrator_->create_subscription<nav_msgs::msg::odometry>(
      "/ground_truth/odom", 10, std::bind(this->read_from_car, this, std::placeholders::_1));
}

void EufsCommunicator::EufsCommunicator::send_to_car() {
  auto msg = std::make_unique<ackermann_msgs::msg::ackermann_drive_stamped>();
  msg->drive.steering_angle = orchestrator_->steering_angle;
  msg->drive.speed = orchestrator_->velocity;
  publisher_->publish(msg);
}

custom_interfaces::msg::vehicle_info EufsCommunicator::read_from_car() { return nullptr; }
