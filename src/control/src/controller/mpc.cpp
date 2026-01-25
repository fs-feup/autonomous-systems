#include "controller/mpc.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

MPC::MPC(const ControlParameters& params) : Controller(params) {
  RCLCPP_INFO(rclcpp::get_logger("mpc"), "Initializing MPC Controller");
  this->solver_ = std::make_shared<AcadosSolver>(params);
}

void MPC::path_callback(const custom_interfaces::msg::PathPointArray& msg) {
  std::vector<double> path_data;
  
  // Convert path points to flat data format
  if (msg.pathpoint_array.size() < 50) {
    for (const auto& point : msg.pathpoint_array) {
      path_data.push_back(point.x);
      path_data.push_back(point.y);
      path_data.push_back(point.v);
    }
    for (size_t i = 0; i < 50 - msg.pathpoint_array.size(); i++) {
      path_data.push_back(0);
      path_data.push_back(0);
      path_data.push_back(0);
    }
  } else {
    // Find the closest point to the vehicle
    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < msg.pathpoint_array.size(); ++i) {
      double dx = msg.pathpoint_array[i].x - this->solver_state_[0];
      double dy = msg.pathpoint_array[i].y - this->solver_state_[1];
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
      }
    }
    
    // Add 50 points starting from closest point
    for (size_t i = 0; i < 50; ++i) {
      size_t idx = (closest_idx + i) % msg.pathpoint_array.size();
      const auto& point = msg.pathpoint_array[idx];
      path_data.push_back(point.x);
      path_data.push_back(point.y);
      path_data.push_back(point.v);
    }
  }
  
  this->solver_->set_path(path_data);
}

void MPC::vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) {
  this->solver_state_[3] = msg.velocity_x;
  this->solver_state_[4] = msg.velocity_y;
  this->solver_state_[5] = msg.angular_velocity;
}

void MPC::vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) {
  this->solver_state_[0] = msg.x;
  this->solver_state_[1] = msg.y;
  this->solver_state_[2] = msg.theta;
}

common_lib::structures::ControlCommand MPC::get_control_command() {
  this->solver_->set_state(this->solver_state_);
  common_lib::structures::ControlCommand command = this->solver_->solve();
  return command;
}