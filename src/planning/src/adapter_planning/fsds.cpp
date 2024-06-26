#include "adapter_planning/fsds.hpp"

#include "planning/planning.hpp"

FsdsAdapter::FsdsAdapter(const PlanningParameters& params) : Planning(params) {
  if (this->planning_config_.simulation_.using_simulated_se_) {
    RCLCPP_WARN(this->get_logger(),
                "FSDS shouldn't be used with simulated State Estimation\n The planning node will "
                "not determine the middle path\n");
    this->fsds_state_subscription_ = this->create_subscription<fs_msgs::msg::GoSignal>(
        "/signal/go", 10,
        std::bind(&FsdsAdapter::mission_state_callback, this, std::placeholders::_1));
    this->fsds_position_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/testing_only/odom", 10,
        std::bind(&FsdsAdapter::pose_callback, this, std::placeholders::_1));
  }
  this->fsds_ebs_publisher_ =
      this->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
  RCLCPP_DEBUG(this->get_logger(), "Planning : FSDS adapter created");
}

void FsdsAdapter::mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  auto mission = msg.mission;
  // map fsds mission to system mission
  this->set_mission(common_lib::competition_logic::fsds_to_system.at(mission));
}

void FsdsAdapter::set_mission_state() {
  RCLCPP_INFO(this->get_logger(), "Planning : Set mission undefined for FSDS");
}

void FsdsAdapter::finish() {
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true;  // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}

void FsdsAdapter::pose_callback(const nav_msgs::msg::Odometry& msg) {
  custom_interfaces::msg::VehicleState pose;
  pose.header = msg.header;
  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll;
  double pitch;
  double yaw;
  m.getRPY(roll, pitch, yaw);
  pose.theta = yaw;
  pose.position.x = msg.pose.pose.position.x;
  pose.position.y = msg.pose.pose.position.y;
  this->vehicle_localization_callback(pose);
}
