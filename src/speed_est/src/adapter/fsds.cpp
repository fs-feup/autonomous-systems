#include "adapter/fsds.hpp"
#include "speed_est/se_node.hpp"

FsdsAdapter::FsdsAdapter(SENode* speed_est) : Adapter(speed_est) {
  this->init();
}

void FsdsAdapter::init() {
  this->fsds_state_subscription_ = this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&FsdsAdapter::mission_state_callback, this, std::placeholders::_1));
  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);

  this->_fs_wheel_speeds_subscription =
      this->node->create_subscription<fs_msgs::msg::WheelStates>(
            "/wheel_states", 10,
            std::bind(&FsdsAdapter::wheel_speeds_subscription_callback, this,
                      std::placeholders::_1));

  this->_fs_imu_subscription = this->node->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&Adapter::imu_subscription_callback, this, std::placeholders::_1));
}

void FsdsAdapter::mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  auto mission = msg.mission;
  // map fsds mission to system mission
  this->node->set_mission(fsdsToSystem.at(mission));
}

void FsdsAdapter::finish() {
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true; // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}

void FsdsAdapter::wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates msg) {
  float steering_angle = (msg.fl_steering_angle + msg.fr_steering_angle) / 2.0;
  this->node->_wheel_speeds_subscription_callback(msg.rl_rpm, msg.fl_rpm, msg.rr_rpm, msg.fr_rpm,
                                                  steering_angle);
}

