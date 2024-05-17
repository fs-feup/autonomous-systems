#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_VEHICLE_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_VEHICLE_HPP_

#include "adapter_planning/adapter.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "std_msgs/msg/string.hpp"

class VehicleAdapter : public Adapter {
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr mission_subscription_;

public:
  explicit VehicleAdapter(Planning* planning);

  void mission_state_callback(const custom_interfaces::msg::OperationalStatus& msg);
  void finish() override;
};

#endif