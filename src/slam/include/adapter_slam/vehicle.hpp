#pragma once

#include "custom_interfaces/msg/operational_status.hpp"
#include "ros_node/slam_node.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

class SLAMNode;

class VehicleAdapter : public SLAMNode {
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr
      _operational_status_subscription_;  ///< Subscriber for operational status

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      _finished_client_;  ///< Client for finished signal

public:
  explicit VehicleAdapter();

  void finish();
};