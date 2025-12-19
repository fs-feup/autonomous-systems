#include "ros_node/ros_node.hpp"

using namespace common_lib::structures;

ControlNode::ControlNode(const ControlParameters& params)
    : Node("control"),
      params_(params),
      _execution_times_(std::make_shared<std::vector<double>>(5, 0.0)),
      control_solver_(controller_map.at(params_.control_solver_)(params)),
      execution_time_pub_(create_publisher<std_msgs::msg::Float64MultiArray>(
          "/control/execution_time", 10)),
      control_timer_(create_wall_timer(std::chrono::milliseconds(this->params_.command_time_interval_),
                              std::bind(&ControlNode::control_timer_callback, this))),
      path_point_array_sub_(create_subscription<custom_interfaces::msg::PathPointArray>(
          params.use_simulated_planning_ ? "/path_planning/mock_path" : "/path_planning/path",
          rclcpp::QoS(10),
          std::bind(&ControlNode::path_callback, this, std::placeholders::_1))) {
  if (!params_.using_simulated_slam_) {
    vehicle_pose_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/state_estimation/vehicle_pose", 10,
        std::bind(&ControlNode::vehicle_pose_callback, this, std::placeholders::_1));
  }
  if (!params_.using_simulated_velocities_) {
    velocity_sub_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/state_estimation/velocities", 10,
        std::bind(&ControlNode::vehicle_state_callback, this, std::placeholders::_1));
  }
}

void ControlNode::control_timer_callback() {
  if (!go_signal_) return;

  rclcpp::Time start = this->now();
  common_lib::structures::ControlCommand command = this->control_solver_->get_control_command();
  double execution_time = (this->now() - start).seconds() * 1000;

  this->publish_command(command);

  this->_execution_times_->at(0) = execution_time;
  std_msgs::msg::Float64MultiArray execution_time_msg;
  execution_time_msg.data = *this->_execution_times_;
  this->execution_time_pub_->publish(execution_time_msg);
  this->control_solver_->publish_solver_data(shared_from_this(), publisher_map_);
}

// This function is called when a new pose is received
void ControlNode::vehicle_pose_callback(const custom_interfaces::msg::Pose& vehicle_pose_msg) {
  this->control_solver_->vehicle_pose_callback(vehicle_pose_msg);
}

void ControlNode::path_callback(const custom_interfaces::msg::PathPointArray& path_msg) {
  this->control_solver_->path_callback(path_msg);
}

void ControlNode::vehicle_state_callback(const custom_interfaces::msg::Velocities& vel_msg) {
  this->control_solver_->vehicle_state_callback(vel_msg);
}