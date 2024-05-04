#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_EUFS_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_EUFS_HPP_

#include "adapter_planning/adapter.hpp"

class EufsAdapter : public Adapter {
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr eufs_state_subscription_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

 public:
  explicit EufsAdapter(Planning* planning);

  void init() override;
  void mission_state_callback(eufs_msgs::msg::CanState msg);
  void set_mission_state(int mission, int state) override;
  void finish() override;
};

#endif