#include "adapter/adapter.hpp"

#include "planning/planning.hpp"

Adapter::Adapter(std::string mode, Planning* planning) {
  this->node = planning;

  if (mode == "eufs") {
    this->eufs_init();
  } else if (mode == "fsds") {
    this->fsds_init();
  } else if (mode == "ads_dv") {
    this->ads_dv_init();
  }
}

void Adapter::eufs_init() {
  this->node->create_subscription<eufs_msgs::msg::CanState>(
      "/ros_can/state", 10, std::bind(&Adapter::eufs_mission_state_callback, this, _1));

  this->eufs_mission_state_client_ =
      this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");

  this->eufs_ebs_client_ = this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/ebs");
}

void Adapter::fsds_init() {
  this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&Adapter::fsds_mission_state_callback, this, std::placeholders::_1));
  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
}

void Adapter::ads_dv_init() {}

void Adapter::eufs_mission_state_callback(eufs_msgs::msg::CanState msg) {
  RCLCPP_INFO(this->node->get_logger(), "I heard: '%d' and '%d'", msg.ami_state, msg.as_state);

  auto mission = msg.ami_state;

  if (mission == eufs_msgs::msg::CanState::AMI_ACCELERATION) {
    this->node->set_mission(Mission::acceleration);
  } else if (mission == eufs_msgs::msg::CanState::AMI_SKIDPAD) {
    this->node->set_mission(Mission::skidpad);
  } else if (mission == eufs_msgs::msg::CanState::AMI_TRACK_DRIVE) {
    this->node->set_mission(Mission::trackdrive);
  } else if (mission == eufs_msgs::msg::CanState::AMI_AUTOCROSS) {
    this->node->set_mission(Mission::autocross);
  }
}

void Adapter::fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  std::string mission = msg.mission;

  if (mission == "acceleration") {
    this->node->set_mission(Mission::acceleration);
  } else if (mission == "skidpad") {
    this->node->set_mission(Mission::skidpad);
  } else if (mission == "trackdrive") {
    this->node->set_mission(Mission::trackdrive);
  } else if (mission == "autocross") {
    this->node->set_mission(Mission::autocross);
  }
}

void Adapter::eufs_set_mission_state(int mission, int state) {
  auto request = std::make_shared<eufs_msgs::srv::SetCanState::Request>();
  request->ami_state = mission;
  request->as_state = state;

  auto result_future = this->eufs_mission_state_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->node->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to call service");
  }
}