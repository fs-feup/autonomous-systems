#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_FSDS_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_FSDS_HPP_

#include "adapter_perception/adapter.hpp"

class Perception;

class FsdsAdapter : public Adapter {
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

 public:
  explicit FsdsAdapter(Perception* perception);

  void init() override;
  void mission_state_callback(const fs_msgs::msg::GoSignal msg);
  void finish() override;
};

#endif