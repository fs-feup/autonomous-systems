#include "adapter_planning/eufs.hpp"

#include "planning/planning.hpp"

EufsAdapter::EufsAdapter(Planning* planning) : Adapter(planning) {
  RCLCPP_INFO(this->node->get_logger(), "EufsAdapter created");
  if (this->node->using_simulated_se_) {
    RCLCPP_INFO(this->node->get_logger(), "Eufs using simulated State Estimation\n");
    this->eufs_pose_subscription_ = this->node->create_subscription<eufs_msgs::msg::CarState>(
        "/odometry_integration/car_state", 10,
        std::bind(&EufsAdapter::pose_callback, this, std::placeholders::_1));
    this->eufs_map_subscription_ =
        this->node->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
            "/ground_truth/track", 10,
            std::bind(&EufsAdapter::map_callback, this, std::placeholders::_1));
  }
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
  this->node->set_mission(
      common_lib::competition_logic::eufs_to_system.at(static_cast<uint16_t>(mission)));
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

void EufsAdapter::pose_callback(const eufs_msgs::msg::CarState& msg) {
  custom_interfaces::msg::VehicleState pose;
  // only gets the x, y, and theta since those are the only ones necessary for planning
  pose.position.x = msg.pose.pose.position.x;
  pose.position.y = msg.pose.pose.position.y;
  pose.theta = atan2(2.0f * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                             msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                     msg.pose.pose.orientation.w * msg.pose.pose.orientation.w +
                         msg.pose.pose.orientation.x * msg.pose.pose.orientation.x -
                         msg.pose.pose.orientation.y * msg.pose.pose.orientation.y -
                         msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
  this->node->vehicle_localization_callback(pose);
}

void EufsAdapter::finish() { std::cout << "Finish undefined for Eufs\n"; }

void EufsAdapter::map_callback(const eufs_msgs::msg::ConeArrayWithCovariance& msg) {
  RCLCPP_INFO(this->node->get_logger(), "Received cones from EUFS");
  custom_interfaces::msg::ConeArray cones;
  for (auto cone : msg.blue_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.yellow_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.orange_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.big_orange_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.unknown_color_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  this->node->track_map_callback(cones);
}