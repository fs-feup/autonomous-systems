#include "adapter/eufs.hpp"

#include "planning/planning.hpp"

EufsAdapter::EufsAdapter(Planning* planning) : Adapter(planning) { this->init(); }

void EufsAdapter::init() {
  this->eufs_state_subscription_ = this->node->create_subscription<eufs_msgs::msg::CanState>(
      "/ros_can/state", 10,
      std::bind(&EufsAdapter::mission_state_callback, this, std::placeholders::_1));

  this->eufs_mission_state_client_ =
      this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");

  this->eufs_ebs_client_ = this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/ebs");
}

void EufsAdapter::mission_state_callback(eufs_msgs::msg::CanState msg) {
  auto mission = msg.ami_state;
  // map eufs mission to system mission
  this->node->set_mission(eufsToSystem.at(static_cast<uint16_t>(mission)));
}

void EufsAdapter::set_mission_state(int mission, int state) {
  auto request = std::make_shared<eufs_msgs::srv::SetCanState::Request>();
  request->ami_state = mission;
  request->as_state = state;

  auto result_future = this->eufs_mission_state_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->node->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to call service");
  }
}

void EufsAdapter::finish() { std::cout << "Finish undefined for Eufs\n"; }
