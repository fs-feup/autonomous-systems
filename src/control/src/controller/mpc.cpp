#include "controller/mpc.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>

#define PATHPOINT_SIZE 4 // x, y, v, orientation
#define PATH_SIZE 31 // number of points in the path horizon

MPC::MPC(const ControlParameters& params) : Controller(params) {
  RCLCPP_INFO(rclcpp::get_logger("mpc"), "Initializing MPC Controller");
  this->solver_ = std::make_shared<AcadosSolver>(params);
}

void add_orientation(std::vector<double>& path_data) {
  // Add orientation for each point
  double prev_x = path_data[0];
  double prev_y = path_data[1];

  double current_x = path_data[PATHPOINT_SIZE];
  double current_y = path_data[PATHPOINT_SIZE + 1];

  double next_x = path_data[2 * PATHPOINT_SIZE];
  double next_y = path_data[2 * PATHPOINT_SIZE + 1];

  // Set orientation for the first point based on the first two points
  double dx = current_x - prev_x;
  double dy = current_y - prev_y;
  double orientation = std::atan2(dy, dx);
  path_data[3] = orientation;

  // Compute orientation for the rest of the points based on three consecutive points for better accuracy
  size_t i = PATHPOINT_SIZE;
  for (; i < path_data.size() - PATHPOINT_SIZE; i += PATHPOINT_SIZE) {
    next_x = path_data[i + PATHPOINT_SIZE];
    next_y = path_data[i + PATHPOINT_SIZE + 1];
    dx = next_x - prev_x;
    dy = next_y - prev_y;
    orientation = std::atan2(dy, dx);
    path_data[i + 3] = orientation;
    prev_x = current_x;
    prev_y = current_y;
    current_x = next_x;
    current_y = next_y;
  }

  // Set orientation for the last point based on the last two points
  dx = next_x - prev_x;
  dy = next_y - prev_y;
  orientation = std::atan2(dy, dx);
  path_data[path_data.size() - 1] = orientation;
}

void MPC::path_callback(const custom_interfaces::msg::PathPointArray& new_msg) {
  std::vector<double> path_data;
  path_data.reserve(PATH_SIZE * PATHPOINT_SIZE); // Reserve space for 50 points (x, y, v, orientation)
  
  // Convert path points to flat data format
  if (new_msg.pathpoint_array.size() < PATH_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc"), "Received path has less than %d points, cannot fill the horizon. Received %zu points.", PATH_SIZE, new_msg.pathpoint_array.size());
    return;
  } else {
    // Find the closest point to the vehicle
    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < new_msg.pathpoint_array.size(); ++i) {
      double dx = new_msg.pathpoint_array[i].x - this->solver_state_[0];
      double dy = new_msg.pathpoint_array[i].y - this->solver_state_[1];
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
    }

    // Start from the previous point for better continuity
    if (closest_idx > 0) {
      double dx = new_msg.pathpoint_array[closest_idx].x - this->solver_state_[0];
      double dy = new_msg.pathpoint_array[closest_idx].y - this->solver_state_[1];
      double dot_product = dx * std::cos(this->solver_state_[2]) + dy * std::sin(this->solver_state_[2]);
      if (dot_product > 0) closest_idx--;
    }
    // Add PATH_SIZE points starting from closest point
    for (size_t i = 0; i < PATH_SIZE; ++i) {
      size_t idx = (closest_idx + i) % new_msg.pathpoint_array.size();
      const auto& point = new_msg.pathpoint_array[idx];
      path_data.push_back(point.x);
      path_data.push_back(point.y);
      path_data.push_back(point.v);
      path_data.push_back(0); // Placeholder for orientation, can be computed if needed
    }
  }

  add_orientation(path_data);
  this->_path_received_ = true;
  this->solver_->set_path(path_data);
}

void MPC::vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& msg) {
  this->solver_state_[3] = msg.velocity_x;
  this->solver_state_[4] = msg.velocity_y;
  this->solver_state_[5] = msg.yaw_rate;
  this->solver_state_[6] = msg.acceleration_x;
  this->solver_state_[7] = msg.acceleration_y;
  this->solver_state_[8] = msg.steering_angle;
  this->solver_state_[9] = msg.fl_rpm;
  this->solver_state_[10] = msg.fr_rpm;
  this->solver_state_[11] = msg.rl_rpm;
  this->solver_state_[12] = msg.rr_rpm;
}

void MPC::vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) {
  this->solver_state_[0] = msg.x;
  this->solver_state_[1] = msg.y;
  this->solver_state_[2] = msg.theta;
}

void MPC::publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  this->solver_->publish_solver_data(node, publisher_map);
}

common_lib::structures::ControlCommand MPC::get_control_command() {
  this->solver_->set_state(this->solver_state_);
  if (!this->_path_received_) {
    return common_lib::structures::ControlCommand(); // Return zero command if path not received yet
  }

  common_lib::structures::ControlCommand command = this->solver_->solve();
  // RCLCPP_INFO(rclcpp::get_logger("mpc"), "Computed control command: throttle_rl=%.3f, throttle_rr=%.3f, steering_angle=%.3f", command.throttle_rl, command.throttle_rr, command.steering_angle);
// 
  // std::vector<common_lib::structures::ControlCommand> full_solution = this->solver_->get_full_solution();
  // std::vector<custom_interfaces::msg::VehicleStateVector> full_horizon = this->solver_->get_full_horizon();
// 
  // RCLCPP_INFO(rclcpp::get_logger("mpc"), "Full State over horizon:");
  // for (size_t i = 0; i < full_horizon.size(); ++i) {
  //   const auto& state = full_horizon[i];
  //   RCLCPP_INFO(rclcpp::get_logger("mpc"), "Stage %zu: v_x=%.3f, v_y=%.3f, yr=%.3f, a_x=%.3f, a_y=%.3f, steering=%.3f, fl=%.3f, fr=%.3f, rl=%.3f, rr=%.3f",
  //               i, state.velocity_x, state.velocity_y, state.yaw_rate, state.acceleration_x, state.acceleration_y, state.steering_angle, state.fl_rpm, state.fr_rpm, state.rl_rpm, state.rr_rpm);
  //   const auto& cmd = full_solution[i];
  //   RCLCPP_INFO(rclcpp::get_logger("mpc"), "Stage %zu: throttle_rl=%.3f, throttle_rr=%.3f, steering_angle=%.3f", i, cmd.throttle_rl, cmd.throttle_rr, cmd.steering_angle);
  // }
  return command;
}