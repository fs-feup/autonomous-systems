#include "communicators/fsds.hpp"

#include <stdio.h>
#include <unistd.h>

#include "custom_interfaces/msg/vcu_command.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "rclcpp/rclcpp.hpp"

FsdsCommunicator::FsdsCommunicator(Orchestrator* orchestrator)
    : Communicator(), orchestrator_(orchestrator) {}

void FsdsCommunicator::FsdsCommunicator::send_to_car(
    const custom_interfaces::msg::VcuCommand msg) {}

void FsdsCommunicator::send_from_car() {
  custom_interfaces::msg::Vcu res;
  this->orchestrator_->publish_info(res);
}
