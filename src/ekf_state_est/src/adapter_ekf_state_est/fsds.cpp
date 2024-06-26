#include "adapter_ekf_state_est/fsds.hpp"

#include "common_lib/competition_logic/mission_logic.hpp"
#include "ros_node/se_node.hpp"

FsdsAdapter::FsdsAdapter(const EKFStateEstParameters& params) : SENode(params) {
  this->fsds_state_subscription_ = this->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&FsdsAdapter::mission_state_callback, this, std::placeholders::_1));
  this->fsds_ebs_publisher_ =
      this->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);

  this->_fs_wheel_speeds_subscription_ = this->create_subscription<fs_msgs::msg::WheelStates>(
      "/wheel_states", 10,
      std::bind(&FsdsAdapter::wheel_speeds_subscription_callback, this, std::placeholders::_1));

  this->_fs_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&FsdsAdapter::imu_subscription_callback, this, std::placeholders::_1));
}

void FsdsAdapter::mission_state_callback(const fs_msgs::msg::GoSignal& msg) {
  RCLCPP_DEBUG(this->get_logger(), "Mission state received: %s", std::string(msg.mission).c_str());
  this->_mission_ = common_lib::competition_logic::fsds_to_system.at(msg.mission);
  this->_go_ = true;
}

void FsdsAdapter::finish() {
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true;  // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}

void FsdsAdapter::wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates& msg) {
  double steering_angle = (msg.fl_steering_angle + msg.fr_steering_angle) / 2.0;
  this->_wheel_speeds_subscription_callback(msg.rl_rpm, msg.fl_rpm, msg.rr_rpm, msg.fr_rpm,
                                            steering_angle, msg.header.stamp);
}
