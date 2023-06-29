#include "communicators/fsds.hpp"

#include <stdio.h>
#include <unistd.h>

#include "custom_interfaces/msg/vehicle_command.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"
#include "rclcpp/rclcpp.hpp"

FsdsCommunicator::FsdsCommunicator(Orchestrator* orchestrator)
    : Communicator(), orchestrator_(orchestrator) {}

void FsdsCommunicator::FsdsCommunicator::send_to_car(
    const custom_interfaces::msg::VehicleCommand msg) {}

void FsdsCommunicator::send_from_car() {
  custom_interfaces::msg::VehicleInfo res;
  this->orchestrator_->publish_info(res);
}
