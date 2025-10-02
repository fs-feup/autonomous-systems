#include "adapter_planning/vehicle.hpp"

#include "planning/planning.hpp"  // Add this line

VehicleAdapter::VehicleAdapter(const PlanningParameters& params) : Planning(params) {
  this->mission_subscription_ =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&VehicleAdapter::mission_state_callback, this, std::placeholders::_1));
  RCLCPP_DEBUG(this->get_logger(), "Planning : Vehicle adapter created");
}

void VehicleAdapter::mission_state_callback(const custom_interfaces::msg::OperationalStatus& msg) {
  this->mission_ = common_lib::competition_logic::Mission(msg.as_mission);
}

void VehicleAdapter::finish() {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Planning : VehicleAdapter::finish()");
}