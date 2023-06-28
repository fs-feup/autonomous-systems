#include <stdio.h>
#include <unistd.h>

#include "communicators/ads-dv.hpp"
#include "communicators/communicator.hpp"
#include "communicators/eufs.hpp"
#include "communicators/fsds.hpp"
#include "custom_interfaces/msg/vehicle_command.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"
#include "rclcpp/rclcpp.hpp"

void Orchestrator::timer_callback() {
  custom_interfaces::msg::VehicleInfo data = this->communicator_->read_from_car();
  publisher_->publish(data);
}

void Orchestrator::command_callback() { this->communicator_->send_to_car(); }

Orchestrator::Orchestrator(std::string mode) : Node("orchestrator") {
  if (mode == "ads-dv") {
    this->communicator_ = new AdsDvCommunicator();
  } else if (mode == "eufs") {
    this->communicator_ = new EufsCommunicator(this);
  } else if (mode == "fsds") {
    this->communicator_ = new FsdsCommunicator();
  } else {
    printf("Invalid mode!\r\n");
  }

  this->publisher_ =
      this->create_publisher<custom_interfaces::msg::VehicleInfo>("vehicle_info", 10);
  this->subscriber_ = this->create_subscription<custom_interfaces::msg::VehicleCommand>(
      "vehicle_command", 10,
      std::bind(&Orchestrator::command_callback, this, std::placeholders::_1));
  this->timer_ = this->create_wall_timer(100ms, std::bind(&Orchestrator::timer_callback, this));
}