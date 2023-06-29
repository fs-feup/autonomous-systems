#include "orchestrator/orchestrator.hpp"

#include "communicators/ads-dv.hpp"
#include "communicators/eufs.hpp"
#include "communicators/fsds.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "rclcpp/rclcpp.hpp"

Orchestrator::Orchestrator(const std::string& mode) : Node("orchestrator") {
  if (mode == "ads-dv") {
    this->communicator_ = new AdsDvCommunicator(this);
  } else if (mode == "eufs") {
    this->communicator_ = new EufsCommunicator(this);
  } else if (mode == "fsds") {
    this->communicator_ = new FsdsCommunicator(this);
  } else {
    printf("Invalid mode!\r\n");
  }

  this->info_publisher_ =
      this->create_publisher<custom_interfaces::msg::Vcu>("vcu", 10);
  this->command_subscriber_ = this->create_subscription<custom_interfaces::msg::VcuCommand>(
      "vcu_command", 10,
      std::bind(&Orchestrator::send_to_car, this, std::placeholders::_1));
}

void Orchestrator::send_to_car(custom_interfaces::msg::VcuCommand msg) {
  this->communicator_->send_to_car(msg);
}

void Orchestrator::publish_info(custom_interfaces::msg::Vcu msg) {
  this->info_publisher_->publish(msg);
}