#include <stdio.h>
#include <unistd.h>

#include <cstdio>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class EufsCommunicator : public Communicator {
 public:
  EufsCommunicator(Orchestrator* orchestrator) {
    this->orchestrator_ = orchestrator;

    publisher_ =
        orchestrator_->create_publisher<ackermann_msgs::msg::ackermann_drive_stamped>("/cmd", 10);
    subscriber_ = orchestrator_->create_subscription<nav_msgs::msg::odometry>(
        "/ground_truth/odom", 10, std::bind(this->read_from_car, this, std::placeholders::_1));
  }

 private:
  rclcpp::Publisher<ackermann_msgs::msg::ackermann_drive_stamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::odometry>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  Orchestrator* orchestrator_;

  void EufsCommunicator::send_to_car() {
    auto msg = std::make_unique<ackermann_msgs::msg::ackermann_drive_stamped>();
    msg->drive.steering_angle = orchestrator_->steering_angle;
    msg->drive.speed = orchestrator_->velocity;
    publisher_->publish(msg);
  }

  custom_interfaces::msg::vehicle_info read_from_car() {}
}
