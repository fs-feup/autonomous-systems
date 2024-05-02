#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_FSDS_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_FSDS_HPP_

#include "adapter_planning/adapter.hpp"

class FsdsAdapter : public Adapter {
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

 public:
  explicit FsdsAdapter(Planning* planning);

  void init() override;
  void mission_state_callback(const fs_msgs::msg::GoSignal msg);
  void set_mission_state(int mission, int state) override;
  void finish() override;
};

#endif