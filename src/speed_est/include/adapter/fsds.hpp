#ifndef SRC_SPEED_EST_INCLUDE_ADAPTER_FSDS_HPP_
#define SRC_SPEED_EST_INCLUDE_ADAPTER_FSDS_HPP_

#include "adapter/adapter.hpp"

class SENode;

class FsdsAdapter : public Adapter {
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _fs_imu_subscription;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr _fs_wheel_speeds_subscription;

 public:
  explicit FsdsAdapter(SENode* speed_est);

  void init() override;
  void mission_state_callback(const fs_msgs::msg::GoSignal msg);
  void finish() override;

  void wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates msg);
};

#endif