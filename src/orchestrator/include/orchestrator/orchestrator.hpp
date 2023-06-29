#ifndef SRC_ORCHESTRATOR_INCLUDE_ORCHESTRATOR_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_ORCHESTRATOR_HPP_

#include <stdio.h>
#include <unistd.h>

#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/gps.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Orchestrator : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::Vcu>::SharedPtr vcu_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::Gps>::SharedPtr gps_publisher_;
  rclcpp::Subscription<custom_interfaces::msg::VcuCommand>::SharedPtr command_subscriber_;
  Communicator* communicator_;

  void send_to_car(custom_interfaces::msg::VcuCommand msg);

 public:
  Orchestrator(const std::string& mode);
  void publish_vcu(custom_interfaces::msg::Vcu msg);
  void publish_imu(custom_interfaces::msg::Imu msg);
  void publish_gps(custom_interfaces::msg::Gps msg);
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_ORCHESTRATOR_HPP_