#ifndef SRC_ORCHESTRATOR_INCLUDE_ORCHESTRATOR_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_ORCHESTRATOR_HPP_

#include <stdio.h>
#include <unistd.h>

#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/vehicle_command.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Orchestrator : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::VehicleInfo>::SharedPtr info_publisher_;
  rclcpp::Subscription<custom_interfaces::msg::VehicleCommand>::SharedPtr command_subscriber_;
  Communicator* communicator_;

  void send_to_car(custom_interfaces::msg::VehicleCommand msg);

 public:
  Orchestrator(const std::string& mode);
  void publish_info(custom_interfaces::msg::VehicleInfo msg);
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_ORCHESTRATOR_HPP_