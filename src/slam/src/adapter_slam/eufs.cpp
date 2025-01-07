#include "adapter_slam/eufs.hpp"

#include <algorithm>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "ros_node/slam_node.hpp"

EufsAdapter::EufsAdapter() : SLAMNode() {
  this->eufs_state_subscription_ = this->create_subscription<eufs_msgs::msg::CanState>(
      "/ros_can/state", 10,
      std::bind(&EufsAdapter::mission_state_callback, this, std::placeholders::_1));

  this->eufs_mission_state_client_ =
      this->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");

  this->eufs_ebs_client_ = this->create_client<eufs_msgs::srv::SetCanState>("/ros_can/ebs");

  if (_use_simulated_perception_) {  // TODO: make this topic a editable parameter
    this->_perception_detections_subscription_ =
        this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
            "/cones", 1,
            std::bind(&EufsAdapter::perception_detections_subscription_callback, this,
                      std::placeholders::_1));
  }
}

void EufsAdapter::mission_state_callback(const eufs_msgs::msg::CanState& msg) {
  RCLCPP_DEBUG(this->get_logger(), "Mission state received: %d", msg.ami_state);
  _mission_ = common_lib::competition_logic::get_mission_from_eufs(msg.ami_state);
  if (msg.as_state == 2) {
    _go_ = true;
  } else {
    _go_ = false;
  }
}

void EufsAdapter::finish() {
  RCLCPP_WARN(this->get_logger(), "TODO: implement mission finished in SE\n");
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
  _perception_subscription_callback(cone_array_msg);
}
