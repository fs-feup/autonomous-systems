#include "adapter_ekf_state_est/eufs.hpp"

#include <algorithm>

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

  if (this->node->_use_simulated_perception_) {  // TODO: make this topic a editable parameter
    this->_perception_detections_subscription_ =
        this->node->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
            "/cones", 10,
            std::bind(&EufsAdapter::perception_detections_subscription_callback, this,
                      std::placeholders::_1));
  }
}

void EufsAdapter::mission_state_callback(eufs_msgs::msg::CanState msg) {
  auto mission = msg.ami_state;
  // map eufs mission to system mission
  RCLCPP_DEBUG(this->node->get_logger(), "Mission state received: %d", mission);
  this->node->set_mission(common_lib::competition_logic::get_mission_from_eufs(mission));
}

void EufsAdapter::finish() {
  RCLCPP_WARN(this->node->get_logger(), "TODO: implement mission finished in SE\n");
}

void EufsAdapter::perception_detections_subscription_callback(
    const eufs_msgs::msg::ConeArrayWithCovariance& msg) {
  custom_interfaces::msg::ConeArray cone_array_msg;
  unsigned int largest_size = static_cast<unsigned int>(
      std::max({msg.big_orange_cones.size(), msg.blue_cones.size(), msg.yellow_cones.size(),
                msg.orange_cones.size(), msg.unknown_color_cones.size()}));
  for (unsigned int i = 0; i < largest_size; i++) {
    if (i < msg.big_orange_cones.size()) {
      auto cone_message = custom_interfaces::msg::Cone();
      cone_message.position.x = msg.big_orange_cones[i].point.x;
      cone_message.position.y = msg.big_orange_cones[i].point.y;
      cone_message.confidence =
          1.0;  // TODO: add confidence to cone message according to covariance
      cone_message.color = common_lib::competition_logic::get_color_string(
          common_lib::competition_logic::Color::ORANGE);
      cone_array_msg.cone_array.push_back(cone_message);
    }
    if (i < msg.blue_cones.size()) {
      auto cone_message = custom_interfaces::msg::Cone();
      cone_message.position.x = msg.blue_cones[i].point.x;
      cone_message.position.y = msg.blue_cones[i].point.y;
      cone_message.confidence = 1.0;
      cone_message.color = common_lib::competition_logic::get_color_string(
          common_lib::competition_logic::Color::BLUE);
      cone_array_msg.cone_array.push_back(cone_message);
    }
    if (i < msg.yellow_cones.size()) {
      auto cone_message = custom_interfaces::msg::Cone();
      cone_message.position.x = msg.yellow_cones[i].point.x;
      cone_message.position.y = msg.yellow_cones[i].point.y;
      cone_message.confidence = 1.0;
      cone_message.color = common_lib::competition_logic::get_color_string(
          common_lib::competition_logic::Color::YELLOW);
      cone_array_msg.cone_array.push_back(cone_message);
    }
    if (i < msg.orange_cones.size()) {
      auto cone_message = custom_interfaces::msg::Cone();
      cone_message.position.x = msg.orange_cones[i].point.x;
      cone_message.position.y = msg.orange_cones[i].point.y;
      cone_message.confidence = 1.0;
      cone_message.color = common_lib::competition_logic::get_color_string(
          common_lib::competition_logic::Color::ORANGE);
      cone_array_msg.cone_array.push_back(cone_message);
    }
    if (i < msg.unknown_color_cones.size()) {
      auto cone_message = custom_interfaces::msg::Cone();
      cone_message.position.x = msg.unknown_color_cones[i].point.x;
      cone_message.position.y = msg.unknown_color_cones[i].point.y;
      cone_message.confidence = 1.0;
      cone_message.color = common_lib::competition_logic::get_color_string(
          common_lib::competition_logic::Color::UNKNOWN);
      cone_array_msg.cone_array.push_back(cone_message);
    }
  }
  cone_array_msg.header.stamp = msg.header.stamp;
  this->node->_perception_subscription_callback(cone_array_msg);
}

void EufsAdapter::wheel_speeds_subscription_callback(
    const eufs_msgs::msg::WheelSpeedsStamped& msg) {
  this->node->_wheel_speeds_subscription_callback(msg.speeds.lb_speed, msg.speeds.lf_speed,
                                                  msg.speeds.rb_speed, msg.speeds.rf_speed,
                                                  msg.speeds.steering, msg.header.stamp);
}