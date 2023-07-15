#ifndef SRC_CAN_INCLUDE_CAN_HPP_
#define SRC_CAN_INCLUDE_CAN_HPP_

#include <stdio.h>
#include <unistd.h>

#include "custom_interfaces/msg/vcu.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Can : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::Vcu>::SharedPtr _publisher;
  rclcpp::Subscription<custom_interfaces::msg::VcuCommand>::SharedPtr _subscription;
  rclcpp::TimerBase::SharedPtr _timer;

  void send_to_car(custom_interfaces::msg::VcuCommand msg);
  void send_from_car();

 public:
  Can();
};

#endif  // SRC_CAN_INCLUDE_CAN_HPP_