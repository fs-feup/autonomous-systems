#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "eufs_msgs/srv/set_can_state.hpp"
#include "eufs_msgs/msg/can_state.hpp"
#include "rclcpp/rclcpp.hpp"

class Planning;

class Adapter {
  Planning* node;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

  void eufs_init();
  void fsds_init();
  void ads_dv_init();

  void eufs_mission_state_callback(eufs_msgs::msg::CanState msg);
  void eufs_set_mission_state(int mission, int state);

 public:
  Adapter(std::string mode, Planning* planning);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
