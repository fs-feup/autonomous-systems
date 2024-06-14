#pragma once

#include "planning/planning.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "std_msgs/msg/string.hpp"

class VehicleAdapter : public Planning {
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr mission_subscription_;

public:
  explicit VehicleAdapter(const PlanningParameters& params);

  void mission_state_callback(const custom_interfaces::msg::OperationalStatus& msg);
  void finish() override;
};
