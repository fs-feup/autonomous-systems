#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "rclcpp/rclcpp.hpp"

class Planning;

class Adapter {
  Planning* node;

  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr eufs_state_subscription_;
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::Vcu>::SharedPtr ads_dv_state_subscription_;

  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  void eufs_init();
  void fsds_init();
  void ads_dv_init();

  void eufs_mission_state_callback(const eufs_msgs::msg::CanState msg);
  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal);
  void ads_dv_mission_state_callback(const custom_interfaces::msg::Vcu msg);
  void eufs_set_mission_state(int mission, int state);

 public:
  Adapter(std::string mode, Planning* planning);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
