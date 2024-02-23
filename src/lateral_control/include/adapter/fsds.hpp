#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_FSDS_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_FSDS_HPP_

#include "adapter/adapter.hpp"

class LateralControl;

class FsdsAdapter : public Adapter {
    rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
    rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;
    rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr fsds_cmd_publisher_;

 public:
    explicit FsdsAdapter(LateralControl* lat_control);

    void init() override;
    void fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg);
    void finish() override;
    void publish_cmd(float acceleration = 0, float braking = 0, float steering = 0) override;
};

#endif