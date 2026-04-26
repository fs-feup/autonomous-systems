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

void limit_velocity_according_to_current(custom_interfaces::msg::PathPointArray& path_msg, double current_velocity, size_t closest_idx) {
  double max_acceleration = 5.0;

  for (size_t i = 0; i < path_msg.pathpoint_array.size(); ++i) {
    size_t idx = (closest_idx + i) % path_msg.pathpoint_array.size();
    double distance = std::sqrt(std::pow(path_msg.pathpoint_array[idx].x - path_msg.pathpoint_array[closest_idx].x, 2) +
                                std::pow(path_msg.pathpoint_array[idx].y - path_msg.pathpoint_array[closest_idx].y, 2));
    double max_velocity = std::sqrt(std::max(0.0, current_velocity * current_velocity + 2 * max_acceleration * distance));
    if (path_msg.pathpoint_array[idx].v > max_velocity) {
      path_msg.pathpoint_array[idx].v = max_velocity;
    }
  }
}

void MPC::create_local_path(custom_interfaces::msg::PathPointArray& path_msg) {
  int path_points_local = 5;
  double x = this->solver_state_[0];
  double y = this->solver_state_[1];
  //std::cout << "State: x:" << x << ", y:" << y << ", v:" << this->solver_state_[3] << ", orientation:" << this->solver_state_[2] << std::endl;
  std::string path_before_local = "Original path: ";
  for (size_t i = 0; i < 10; ++i) {
    path_before_local += "(" + std::to_string(path_msg.pathpoint_array[i].x) + ", " + std::to_string(path_msg.pathpoint_array[i].y) + ", " + std::to_string(path_msg.pathpoint_array[i].v) + "," + std::to_string(path_msg.pathpoint_array[i].orientation) + "),";
  }
  //std::cout << path_before_local << std::endl;

  custom_interfaces::msg::PathPoint tenth_point = path_msg.pathpoint_array[path_points_local];

  // Remove first points
  path_msg.pathpoint_array.erase(path_msg.pathpoint_array.begin(), path_msg.pathpoint_array.begin() + path_points_local);

  double dx = tenth_point.x - x;
  double dy = tenth_point.y - y;
  double distance = std::sqrt(dx * dx + dy * dy);
  double dv = tenth_point.v - this->solver_state_[3];
  
  std::vector<custom_interfaces::msg::PathPoint> interpolated_points;
  for (int i = 0; i < path_points_local; ++i) {
    if (distance > 1e-6) {
      double ratio = i / static_cast<double>(path_points_local);
      custom_interfaces::msg::PathPoint new_point;
      new_point.x = x + ratio * dx;
      new_point.y = y + ratio * dy;
      new_point.v = this->solver_state_[3] + ratio * dv;
      new_point.orientation = this->solver_state_[2];
      interpolated_points.push_back(new_point);
    }
  }
  
  path_msg.pathpoint_array.insert(path_msg.pathpoint_array.begin(), 
                                   interpolated_points.begin(),
                                   interpolated_points.end());

  // DEBUG PRINT
  std::string path_after_local = "Local path: ";
  for (int i=0; i<10; ++i) {
    path_after_local += "(" + std::to_string(path_msg.pathpoint_array[i].x) + ", " + std::to_string(path_msg.pathpoint_array[i].y) + ", " + std::to_string(path_msg.pathpoint_array[i].v) + "," + std::to_string(path_msg.pathpoint_array[i].orientation) + "),";
  }
  //std::cout << path_after_local << std::endl;

  double diff_x = path_msg.pathpoint_array[0].x - x;
  double diff_y = path_msg.pathpoint_array[0].y - y;
  double diff_v = path_msg.pathpoint_array[0].v - this->solver_state_[3];
  double diff_orientation = path_msg.pathpoint_array[0].orientation - this->solver_state_[2];
  if (std::fabs(diff_x) > 0.01 || std::abs(diff_y) > 0.01 || std::abs(diff_v) > 0.01 || std::abs(diff_orientation) > 0.01) {
    std::cout << "CRAZYERROR1" << std::endl;
  }
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

void MPC::set_path_in_solver() {
  custom_interfaces::msg::PathPointArray msg_copy = this->latest_path_;
  std::vector<double> path_data;
  unsigned int received_point_count = msg_copy.pathpoint_array.size(); // +1 for the interpolated point
  path_data.reserve(received_point_count * PATHPOINT_SIZE); // Reserve space for 50 points (x, y, v, orientation)

  // Convert path points to flat data format
  if (received_point_count < PATH_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc"), "Received path has less than %d points, cannot fill the horizon. Received %zu points.", PATH_SIZE, msg_copy.pathpoint_array.size());
    return;
  } else {
    // Find the closest point to the vehicle
    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < msg_copy.pathpoint_array.size(); ++i) {
      double dx = msg_copy.pathpoint_array[i].x - this->solver_state_[0];
      double dy = msg_copy.pathpoint_array[i].y - this->solver_state_[1];
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
    }

    // Start from the previous point for better continuity
    if (closest_idx > 0) {
      double dx = msg_copy.pathpoint_array[closest_idx].x - this->solver_state_[0];
      double dy = msg_copy.pathpoint_array[closest_idx].y - this->solver_state_[1];
      double dot_product = dx * std::cos(this->solver_state_[2]) + dy * std::sin(this->solver_state_[2]);
      if (dot_product > 0) closest_idx--;
    }

//    // Insert interpolated point at closest position on segment
//    if (closest_idx < msg_copy.pathpoint_array.size() - 1) {
//      const auto& p1 = msg_copy.pathpoint_array[closest_idx];
//      const auto& p2 = msg_copy.pathpoint_array[closest_idx + 1];
//      
//      double dx = p2.x - p1.x;
//      double dy = p2.y - p1.y;
//      double segment_length_sq = dx * dx + dy * dy;
//      
//      if (segment_length_sq > 1e-6) {
//        double car_x = this->solver_state_[0];
//        double car_y = this->solver_state_[1];
//        double t = ((car_x - p1.x) * dx + (car_y - p1.y) * dy) / segment_length_sq;
//        t = std::max(0.0, std::min(1.0, t));
//        
//        auto interpolated_point = custom_interfaces::msg::PathPoint();
//        interpolated_point.x = p1.x + t * dx;
//        interpolated_point.y = p1.y + t * dy;
//        interpolated_point.v = this->solver_state_[3]; // current velocity_x
//        interpolated_point.orientation = this->solver_state_[2]; // current theta
//        
//        // Insert at the closest index position
//        msg_copy.pathpoint_array.insert(msg_copy.pathpoint_array.begin() + closest_idx + 1, interpolated_point);
//      }
//    }
    // Create new path starting from the closest point
    custom_interfaces::msg::PathPointArray new_path;
    for (size_t i = 0; i < received_point_count; ++i) {
      size_t idx = (closest_idx + i) % msg_copy.pathpoint_array.size();
      new_path.pathpoint_array.push_back(msg_copy.pathpoint_array[idx]);
    }

    create_local_path(new_path);

    limit_velocity_according_to_current(new_path, this->solver_state_[3], 0);

    // Add PATH_SIZE points starting from closest point
    for (size_t i = 0; i < received_point_count; ++i) {
      size_t idx = (i) % new_path.pathpoint_array.size();
      const auto& point = new_path.pathpoint_array[idx];
      path_data.push_back(point.x);
      path_data.push_back(point.y);
      path_data.push_back(point.v);
      path_data.push_back(point.orientation);
    }
  }

  this->solver_->set_path(path_data);
}

void MPC::path_callback(const custom_interfaces::msg::PathPointArray& new_msg) {
  this->_path_received_ = true;
  this->latest_path_ = new_msg;
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

  this->set_path_in_solver();

  common_lib::structures::ControlCommand command = this->solver_->solve();
  // DEBUG PRINT
  // std::cout << "Computed control command: throttle_rl=" << command.throttle_rl << ", throttle_rr=" << command.throttle_rr << ", steering_angle=" << command.steering_angle << std::endl;
  // std::cout << "Velocities: v_x=" << this->solver_state_[3] << ", v_y=" << this->solver_state_[4] << ", yaw_rate=" << this->solver_state_[5] << std::endl;
  std::vector<common_lib::structures::ControlCommand> full_solution = this->solver_->get_full_solution();
  std::vector<custom_interfaces::msg::VehicleStateVector> full_horizon = this->solver_->get_full_horizon();

  std::string stat_over_horizon = "";
  for (size_t i = 0; i < 10/*full_horizon.size()*/; ++i) {
    const auto& state = full_horizon[i];
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Stage %zu: x=%.3f, y=%.3f, theta=%.3f, v_x=%.3f, v_y=%.3f, yr=%.3f, a_x=%.3f, a_y=%.3f, steering=%.3f, fl=%.3f, fr=%.3f, rl=%.3f, rr=%.3f \n",
             i, state.x, state.y, state.orientation, state.velocity_x, state.velocity_y, state.yaw_rate, state.acceleration_x, state.acceleration_y, state.steering_angle, state.fl_rpm, state.fr_rpm, state.rl_rpm, state.rr_rpm);
    stat_over_horizon += buffer;
  }
  // DEBUG PRINT
  //std::cout << "Full State over horizon: \n " << stat_over_horizon.c_str() << std::endl;
  std::string command_over_horizon = "";
  for (size_t i = 0; i < 10/*full_horizon.size()*/; ++i) { 
    const auto& cmd = full_solution[i];
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Stage %zu: throttle_rl=%.3f, throttle_rr=%.3f, steering_angle=%.3f \n", i, cmd.throttle_rl, cmd.throttle_rr, cmd.steering_angle);
    command_over_horizon += buffer;
  }
  // DEBUG PRINT
  //std::cout << "Full Command over horizon: \n " << command_over_horizon.c_str() << std::endl;
  return command;
}