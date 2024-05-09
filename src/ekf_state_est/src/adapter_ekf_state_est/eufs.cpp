#include "adapter_ekf_state_est/eufs.hpp"

#include "common_lib/competition_logic/mission_logic.hpp"
#include "ros_node/se_node.hpp"

EufsAdapter::EufsAdapter(std::shared_ptr<SENode> se_node) : Adapter(se_node) { this->init(); }

void EufsAdapter::init() {
  this->eufs_state_subscription_ = this->node->create_subscription<eufs_msgs::msg::CanState>(
      "/ros_can/state", 10,
      std::bind(&EufsAdapter::mission_state_callback, this, std::placeholders::_1));

  this->eufs_mission_state_client_ =
      this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");

  this->eufs_ebs_client_ = this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/ebs");

  this->_eufs_wheel_speeds_subscription =
      this->node->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
          "/ros_can/wheel_speeds",
          rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
          std::bind(&EufsAdapter::wheel_speeds_subscription_callback, this, std::placeholders::_1));

  this->_imu_subscription_ = this->node->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&Adapter::imu_subscription_callback, this, std::placeholders::_1));
}

void EufsAdapter::mission_state_callback(eufs_msgs::msg::CanState msg) {
  auto mission = msg.ami_state;
  // map eufs mission to system mission
  this->node->set_mission(common_lib::competition_logic::eufs_to_system.at(mission));
}

void EufsAdapter::finish() { std::cout << "Finish undefined for Eufs\n"; }

void EufsAdapter::wheel_speeds_subscription_callback(
    const eufs_msgs::msg::WheelSpeedsStamped& msg) {
  this->node->_wheel_speeds_subscription_callback(msg.speeds.lb_speed, msg.speeds.lf_speed,
                                                  msg.speeds.rb_speed, msg.speeds.rf_speed,
                                                  msg.speeds.steering, msg.header.stamp);
}