#pragma once

#include "adapter_control/adapter.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "node_/node_control.hpp"

class Control;

class VehicleAdapter : public Adapter {
 protected:
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr go_sub;
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_pub_;

 public:
  explicit VehicleAdapter(Control *control);
  void publish_cmd(float acceleration, float steering) override;
  void finish() override;
};
