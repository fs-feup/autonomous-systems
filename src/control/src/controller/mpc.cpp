#include "controller/mpc.hpp"
#include "local_pather/spline.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

#define PATHPOINT_SIZE 4 // x, y, v, orientation
#define MIN_PATH_SIZE 5 // number of points in the path horizon

MPC::MPC(const ControlParameters& params) : Controller(params) {
  RCLCPP_INFO(rclcpp::get_logger("mpc"), "Initializing MPC Controller");
  this->solver_ = std::make_shared<AcadosSolver>(params);
  this->local_pather_ = local_pather_map.at("interpolator")(params);
  this->path_data.resize((this->params_->mpc_prediction_horizon_steps_ + 1) * PATHPOINT_SIZE);
}

void print_path(custom_interfaces::msg::PathPointArray& path) {
  std::ios old_state(nullptr);
  old_state.copyfmt(std::cout);
  std::cout << std::fixed << std::setprecision(5);

  for (size_t i = 0; i < path.pathpoint_array.size(); ++i) {
    const auto& point = path.pathpoint_array[i];
    std::cout << "(" << point.x << ", " << point.y <<  " , "<< point.v <<" , " <<point.orientation << "),";
  }
  std::cout << std::endl;

  std::cout.copyfmt(old_state);
}

unsigned int get_s_index(std::vector<double>& s, double query_s) {
  // Simple linear search (can be optimized with binary search if needed)
  for (size_t i = 0; i < s.size() - 1; ++i) {
    if (s[i] > query_s) {
      return i - 1;
    }
  }
  return s.size() - 2; // Return the last valid index if out of bounds
}

double acceleration_for_segment(unsigned int start_idx, std::vector<double>& v, std::vector<double>& s) {
  double distance = s[start_idx + 1] - s[start_idx];
  double v_start = v[start_idx];
  double v_end = v[start_idx + 1];

  if (distance > 0.01) {
    return (v_end * v_end - v_start * v_start) / (2 * distance);
  } else {
    return 0.0; // Avoid division by zero, treat as no acceleration
  }
}

void MPC::resample_path_with_spline(custom_interfaces::msg::PathPointArray& path_msg) {
  if (path_msg.pathpoint_array.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MPC"), "Received empty path, should at least have the car's position as the first point");
    return;
  }

  // Default to just breaking until stopped
  if (path_msg.pathpoint_array.size() < 3) {
    double dt = this->params_->mpc_prediction_horizon_seconds_ / static_cast<double>(this->params_->mpc_prediction_horizon_steps_);
    double max_breaking_acceleration = 3.5; // m/s^2
    double max_yaw_acceleration = 1.0; // rad/s^2
    double current_v = path_msg.pathpoint_array[0].v;
    double current_yaw_rate = this->solver_state_[5];
    double time_to_stop = std::max(std::fabs(current_v / max_breaking_acceleration), std::fabs(current_yaw_rate / max_yaw_acceleration));

    double effective_acceleration = time_to_stop == 0 ? 0 : current_v / time_to_stop;
    double effective_yaw_acceleration = time_to_stop == 0 ? 0 : current_yaw_rate / time_to_stop;

    double current_x = path_msg.pathpoint_array[0].x;
    double current_y = path_msg.pathpoint_array[0].y;
    double current_yaw = path_msg.pathpoint_array[0].orientation;
    double yaw_acceleration = current_yaw_rate / time_to_stop;

    path_msg.pathpoint_array.clear();

    for (int i = 0; i <= this->params_->mpc_prediction_horizon_steps_; ++i) {
      custom_interfaces::msg::PathPoint p;
      p.x = current_x;
      p.y = current_y;
      p.orientation = current_yaw;
      p.v = current_v;
      path_msg.pathpoint_array.push_back(p);

      // Update
      double next_v = current_v - effective_acceleration * dt;
      next_v = (current_v >= 0.0) ? std::max(0.0, next_v) : std::min(0.0, next_v);
      double next_yaw_rate = current_yaw_rate - effective_yaw_acceleration * dt;
      next_yaw_rate = (current_yaw_rate >= 0.0) ? std::max(0.0, next_yaw_rate) : std::min(0.0, next_yaw_rate);
      double average_v = (current_v + next_v) / 2.0;
      double average_yaw_rate = (current_yaw_rate + next_yaw_rate) / 2.0;

      current_x += std::cos(current_yaw) * average_v * dt;
      current_y += std::sin(current_yaw) * average_v * dt;
      current_yaw += average_yaw_rate * dt;
      current_v = next_v;
    }
    return;
  }

  // --- 1. Extract cumulative distance (s) from your existing local path ---
  std::vector<double> s, x, y, velocities;
  double current_s = 0.0;

  double car_x = path_msg.pathpoint_array[0].x;
  double car_y = path_msg.pathpoint_array[0].y;
  double car_v = path_msg.pathpoint_array[0].v;
  double car_yaw = path_msg.pathpoint_array[0].orientation;

  // Point 0: The actual car
  s.push_back(0.0);
  x.push_back(car_x);
  y.push_back(car_y);
  velocities.push_back(car_v);

  double dist_to_next = std::hypot(path_msg.pathpoint_array[1].x - car_x, 
                                   path_msg.pathpoint_array[1].y - car_y);
  double lookahead_dist = 0.1 * dist_to_next;

  // Interpolate the velocity assuming constant acceleration over the segment
  double next_velocity = path_msg.pathpoint_array[1].v;
  double ghost_v = car_v;
  if (dist_to_next > 0.0) {
    double acceleration = (next_velocity * next_velocity - car_v * car_v) / (2.0 * dist_to_next);
    double ghost_v_sq = car_v * car_v + 2.0 * acceleration * lookahead_dist;
    ghost_v = std::sqrt(std::max(0.0, ghost_v_sq));
  }

  // Point 1: THE GHOST POINT
  current_s += lookahead_dist;
  s.push_back(current_s);
  x.push_back(car_x + std::cos(car_yaw) * lookahead_dist);
  y.push_back(car_y + std::sin(car_yaw) * lookahead_dist);
  velocities.push_back(ghost_v);

  // Point 2 to N: The rest of the macro path
  for (size_t i = 1; i < path_msg.pathpoint_array.size(); ++i) {
    double dx = path_msg.pathpoint_array[i].x - x.back();
    double dy = path_msg.pathpoint_array[i].y - y.back();
    double dist = std::hypot(dx, dy);
   
    // Ensure the global points are far enough past the Ghost Point to avoid NaN crashes
    if (dist > 0.1) {
        current_s += dist;
        s.push_back(current_s);
        x.push_back(path_msg.pathpoint_array[i].x);
        y.push_back(path_msg.pathpoint_array[i].y);
        velocities.push_back(path_msg.pathpoint_array[i].v);
    }
  }

  // --- 2. Fit the splines ---
  tk::spline spline_x, spline_y;

  // FORCE the spline's starting derivative to perfectly match the car's exact unit vector
  spline_x.set_boundary(tk::spline::first_deriv, std::cos(car_yaw), tk::spline::second_deriv, 0.0);
  spline_y.set_boundary(tk::spline::first_deriv, std::sin(car_yaw), tk::spline::second_deriv, 0.0);

  spline_x.set_points(s, x);
  spline_y.set_points(s, y);

  // --- 3. Sample the splines based on MPC dt ---
  std::vector<custom_interfaces::msg::PathPoint> resampled_path;
  int N = this->params_->mpc_prediction_horizon_steps_;
    
  current_s = 0.0;
  double current_v = car_v;
  double max_s = s.back();
  double full_dt = this->params_->mpc_prediction_horizon_seconds_ / static_cast<double>(N);

  for (int i = 0; i <= N; ++i) {
    custom_interfaces::msg::PathPoint p;
    
    // Clamp to prevent querying outside the spline bounds
    p.x = spline_x(current_s);
    p.y = spline_y(current_s);

    // Analytically compute perfectly smooth yaw from the geometry tangent
    double dx_ds = spline_x.deriv(1, current_s);
    double dy_ds = spline_y.deriv(1, current_s);
    p.orientation = std::atan2(dy_ds, dx_ds);
    p.v = current_v;
    resampled_path.push_back(p);

    // --- FINAL 2nd-Order Kinematic Integration ---
    unsigned int segment_start_idx = get_s_index(s, current_s);
    double acceleration = acceleration_for_segment(segment_start_idx, velocities, s);
    double end_of_segment_s = s[segment_start_idx + 1];

    double remaining_dt = full_dt;
    double remaining_dt2 = remaining_dt * remaining_dt;
    double ds = (current_v * remaining_dt) + (0.5 * acceleration * remaining_dt2);

    while (current_s + ds > end_of_segment_s) {
      double delta_s = end_of_segment_s - current_s;
      double average_speed = (current_v + velocities[segment_start_idx + 1]) / 2.0;
      double dt_until_segment_end = (average_speed > 1e-3) ? (delta_s / average_speed) : remaining_dt;
      
      current_s = end_of_segment_s;
      current_v = velocities[segment_start_idx + 1];
      segment_start_idx++;

      remaining_dt -= dt_until_segment_end;
      remaining_dt2 = remaining_dt * remaining_dt;
      end_of_segment_s = s[segment_start_idx + 1];
      acceleration = acceleration_for_segment(segment_start_idx, velocities, s);
      ds = (current_v * remaining_dt) + (0.5 * acceleration * remaining_dt2);
    }

    current_v += acceleration * remaining_dt;

    // 3. SAFETY CLAMPS
    // Prevent moving backward
    ds = std::max(ds, 0.0); 

    // Step forward, but STRICTLY LOCK to the end of the macro path
    current_s = std::min(current_s + ds, max_s); 
  }

  // --- 4. Overwrite the original path ---
  path_msg.pathpoint_array = resampled_path;
}

void MPC::limit_velocity_according_to_current(custom_interfaces::msg::PathPointArray& path_msg) {
  double max_acceleration = 10.0;

  if (path_msg.pathpoint_array.empty()) {
    return;
  }

  // Limit velocity segment-by-segment using the previous point as the starting state
  // Start from the vehicle's current pose (front of the path) and current_velocity
  custom_interfaces::msg::PathPoint prev_point = path_msg.pathpoint_array.front();
  double prev_velocity = prev_point.v;

  for (size_t i = 0; i < path_msg.pathpoint_array.size(); ++i) {
    double dx = path_msg.pathpoint_array[i].x - prev_point.x;
    double dy = path_msg.pathpoint_array[i].y - prev_point.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double max_velocity = std::sqrt(std::max(0.0, prev_velocity * prev_velocity + 2 * max_acceleration * distance));
    if (path_msg.pathpoint_array[i].v > max_velocity) {
      path_msg.pathpoint_array[i].v = max_velocity;
    }

    // Use the velocity and point as the previous for the next iteration
    prev_velocity = path_msg.pathpoint_array[i].v;
    prev_point = path_msg.pathpoint_array[i];
  }
}

unsigned int MPC::compute_starting_index() {
  custom_interfaces::msg::PathPointArray msg_copy = this->latest_path_;

  size_t closest_idx = -1;

  // Find the closest point to the vehicle
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
  if (closest_idx > 0 && closest_idx < msg_copy.pathpoint_array.size() - 1) {
    double prev_dx = msg_copy.pathpoint_array[closest_idx - 1].x - this->solver_state_[0];
    double prev_dy = msg_copy.pathpoint_array[closest_idx - 1].y - this->solver_state_[1];
    double next_dx = msg_copy.pathpoint_array[closest_idx + 1].x - this->solver_state_[0];
    double next_dy = msg_copy.pathpoint_array[closest_idx + 1].y - this->solver_state_[1];
    double next_sq_distance = next_dx * next_dx + next_dy * next_dy;
    double prev_sq_distance = prev_dx * prev_dx + prev_dy * prev_dy;
    if (prev_sq_distance < next_sq_distance) closest_idx--;
  }

  return closest_idx;
}

void MPC::create_local_path(custom_interfaces::msg::PathPointArray& path_msg) {

  double x = this->solver_state_[0];
  double y = this->solver_state_[1];
  // DEBUG STRING
  this->path_before_local = "Path received by MPC: ";
  for (size_t i = 0; i < 10; ++i) {
    this->path_before_local += "(" + std::to_string(path_msg.pathpoint_array[i].x) + ", " + std::to_string(path_msg.pathpoint_array[i].y) + "), ";// + std::to_string(path_msg.pathpoint_array[i].v) + "," + std::to_string(path_msg.pathpoint_array[i].orientation) + "),";
  }

  custom_interfaces::msg::VehicleStateVector current_state;
  current_state.x = this->solver_state_[0];
  current_state.y = this->solver_state_[1];
  current_state.orientation = this->solver_state_[2];
  current_state.velocity_x = this->solver_state_[3];
  this->local_pather_->create_local_path(path_msg, current_state);

  // DEBUG STRING
  this->local_path_debug = "Local path: x: " + std::to_string(x) + ", y: " + std::to_string(y)  + "\n";
  for (int i=0; i<10; ++i) {
    this->local_path_debug += "(" + std::to_string(path_msg.pathpoint_array[i].x) + ", " + std::to_string(path_msg.pathpoint_array[i].y) + "), ";// + std::to_string(path_msg.pathpoint_array[i].v) + "," + std::to_string(path_msg.pathpoint_array[i].orientation) + "),";
  }

  double diff_x = path_msg.pathpoint_array[0].x - x;
  double diff_y = path_msg.pathpoint_array[0].y - y;
  double diff_v = path_msg.pathpoint_array[0].v - this->solver_state_[3];
  double diff_orientation = path_msg.pathpoint_array[0].orientation - this->solver_state_[2];
  if (std::fabs(diff_x) > 0.01 || std::abs(diff_y) > 0.01 || std::abs(diff_v) > 0.01 || std::abs(diff_orientation) > 0.01) {
    RCLCPP_ERROR(rclcpp::get_logger("MPC"), "Local path's first point should be equal to car state. This can cause instability. Diff x: %.3f, Diff y: %.3f, Diff v: %.3f, Diff orientation: %.3f", diff_x, diff_y, diff_v, diff_orientation);
  }
}

void MPC::set_path_in_solver() {
  unsigned int starting_idx = this->compute_starting_index();

  size_t received_point_count = this->latest_path_.pathpoint_array.size();
  size_t limit = received_point_count;

  if (this->latest_path_.is_map_closed) {
    limit = starting_idx + received_point_count;
  }

  // Create new path starting from the closest point
  custom_interfaces::msg::PathPointArray new_path;
  for (size_t i = starting_idx; i < limit; ++i) {
    new_path.pathpoint_array.push_back(this->latest_path_.pathpoint_array[i % received_point_count]);
  }
  
  // std::cout << "Latest: \n";
  // print_path(this->latest_path_);

  this->create_local_path(new_path);
  // std::cout << "Local: \n";
  // print_path(new_path);

  this->limit_velocity_according_to_current(new_path);
  // std::cout << "Limited: \n";
  // print_path(new_path);

  this->resample_path_with_spline(new_path);
  // std::cout << "Sampled: \n";
  // print_path(new_path);

  // Add path_size points starting from closest point
  for (size_t i = 0; i <= this->params_->mpc_prediction_horizon_steps_; ++i) {
    size_t idx = (i) % new_path.pathpoint_array.size();
    const auto& point = new_path.pathpoint_array[idx];
    this->path_data[PATHPOINT_SIZE*i] = point.x;
    this->path_data[PATHPOINT_SIZE*i + 1] = point.y;
    this->path_data[PATHPOINT_SIZE*i + 2] = point.v;
    this->path_data[PATHPOINT_SIZE*i + 3] = point.orientation;
  }
  this->solver_->set_path(this->path_data);
}

void MPC::path_callback(const custom_interfaces::msg::PathPointArray& new_msg) {
  if (new_msg.pathpoint_array.size() < MIN_PATH_SIZE) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc"), "Received path has less than %d points, will be discarded. Received %zu points.", MIN_PATH_SIZE, new_msg.pathpoint_array.size());
    return;
  }

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

bool MPC::stopping_the_car() {
  double speed_threshold = 0.4; // m/s

  // Check current speed
  if (std::fabs(this->solver_state_[3]) > speed_threshold) {
    return false;
  }

  // Check average goal speed over the horizon
  double goal_speed_average = 0;
  for (size_t i = 0; i <= this->params_->mpc_prediction_horizon_steps_; ++i) {
    goal_speed_average += this->path_data[PATHPOINT_SIZE*i + 2];
  }
  goal_speed_average /= (this->params_->mpc_prediction_horizon_steps_ + 1);
  if (goal_speed_average > speed_threshold) {
    return false;
  }
  
  // Check if speed is tending towards 0: calculate average speed goal variation
  double speed_variation_average = 0;
  for (size_t i = 0; i < this->params_->mpc_prediction_horizon_steps_; ++i) {
    double current_goal_speed = this->path_data[PATHPOINT_SIZE*i + 2];
    double next_goal_speed = this->path_data[PATHPOINT_SIZE*(i+1) + 2];
    speed_variation_average += (next_goal_speed - current_goal_speed);
  }
  speed_variation_average /= this->params_->mpc_prediction_horizon_steps_;
  double current_velocity = this->solver_state_[3];
  double allowed_negative_speed_due_to_noise = -0.1;
  if (speed_variation_average > 0 && current_velocity > allowed_negative_speed_due_to_noise) {
    return false;
  }

  return true;
}

common_lib::structures::ControlCommand MPC::get_control_command() {
  if (!this->_path_received_) {
    return common_lib::structures::ControlCommand(); // Return zero command if path not received yet
  }

  this->set_path_in_solver();

  if (this->stopping_the_car()) {
    common_lib::structures::ControlCommand stop_command; // all 0, maybe dangerous if the steering isn't at 0, TODO: reconsider
    return stop_command;
  }

  this->solver_->set_state(this->solver_state_);
  int solver_status = 0;
  common_lib::structures::ControlCommand command = this->solver_->solve(&solver_status);

  // DEBUG STRING
  double x = this->solver_state_[0];
  double y = this->solver_state_[1];
  this->current_state = "State: x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", orientation: " + std::to_string(this->solver_state_[2]) + ", vx: " + std::to_string(this->solver_state_[3]) + ", vy: " + std::to_string(this->solver_state_[4]) + ", yaw_rate: " + std::to_string(this->solver_state_[5]);
  this->computed_command = "Computed control command: throttle_rl=" + std::to_string(command.throttle_rl) + ", throttle_rr=" + std::to_string(command.throttle_rr) + ", steering_angle=" + std::to_string(command.steering_angle);

  // DEBUG STRING
  std::vector<custom_interfaces::msg::VehicleStateVector> full_horizon = this->solver_->get_full_horizon();
  this->solver_state_over_horizon = "Full State over horizon: \n ";
  for (size_t i = 0; i < full_horizon.size(); ++i) {
    const auto& state = full_horizon[i];
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Stage %zu: x=%.3f, y=%.3f, theta=%.3f, v_x=%.3f, v_y=%.3f, yr=%.3f, a_x=%.3f, a_y=%.3f, steering=%.3f, fl=%.3f, fr=%.3f, rl=%.3f, rr=%.3f \n",
             i, state.x, state.y, state.orientation, state.velocity_x, state.velocity_y, state.yaw_rate, state.acceleration_x, state.acceleration_y, state.steering_angle, state.fl_rpm, state.fr_rpm, state.rl_rpm, state.rr_rpm);
    this->solver_state_over_horizon += buffer;
  }

  // DEBUG STRING
  std::vector<common_lib::structures::ControlCommand> full_solution = this->solver_->get_full_solution();
  this->solver_command_over_horizon = "Commands over horizon: \n ";
  for (size_t i = 0; i < full_horizon.size() - 1; ++i) { 
    const auto& cmd = full_solution[i];
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Stage %zu: throttle_rl=%.3f, throttle_rr=%.3f, steering_angle=%.3f \n", i, cmd.throttle_rl, cmd.throttle_rr, cmd.steering_angle);
    this->solver_command_over_horizon += buffer;
  }

  if (solver_status < ACADOS_SUCCESS) {
    this->print_debug_info();
  }

  return command;
}

void MPC::print_debug_info() {
  std::cout << this->path_before_local << std::endl;
  std::cout << this->local_path_debug << std::endl;
  std::cout << this->current_state << std::endl;
  std::cout << this->computed_command << std::endl;
  std::cout << this->solver_state_over_horizon << std::endl;
  std::cout << this->solver_command_over_horizon << std::endl;
}