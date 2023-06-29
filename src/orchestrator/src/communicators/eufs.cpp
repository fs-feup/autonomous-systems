#include "communicators/eufs.hpp"

#include <stdio.h>
#include <unistd.h>

#include <functional>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

EufsCommunicator::EufsCommunicator(Orchestrator* orchestrator)
    : Communicator() {
  this->orchestrator_ = orchestrator;
  this->sim_publisher_ =
      this->orchestrator_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
  this->sim_subscriber_ = this->orchestrator_->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth/odom", 10,
      std::bind(&EufsCommunicator::send_from_car, this, std::placeholders::_1));
}

void EufsCommunicator::send_to_car(const custom_interfaces::msg::VcuCommand msg) {
  auto command = ackermann_msgs::msg::AckermannDriveStamped();
  command.drive.steering_angle = msg.steering_angle_request;
  command.drive.speed = msg.axle_speed_request;
  this->sim_publisher_->publish(command);
}

void EufsCommunicator::send_from_car(const nav_msgs::msg::Odometry msg) {
  custom_interfaces::msg::Vcu res;
  this->orchestrator_->publish_info(res);
}
