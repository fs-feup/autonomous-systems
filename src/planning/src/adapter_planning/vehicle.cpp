#include "adapter_planning/vehicle.hpp"

#include "planning/planning.hpp"  // Add this line

VehicleAdapter::VehicleAdapter(std::shared_ptr<Planning> planning) : Adapter(planning) {
  this->mission_subscription_ =
      this->node->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&VehicleAdapter::mission_state_callback, this, std::placeholders::_1));
}

void VehicleAdapter::mission_state_callback(const custom_interfaces::msg::OperationalStatus& msg) {
  this->node->mission = common_lib::competition_logic::Mission(msg.as_mission);
}

void VehicleAdapter::finish() { std::cout << "VehicleAdapter::finish()" << std::endl; }