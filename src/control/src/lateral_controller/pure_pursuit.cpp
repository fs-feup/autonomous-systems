#include "lateral_controller/pure_pursuit.hpp"

using namespace common_lib::structures;

/**
 * @brief Pure Pursuit class Constructor
 *
 */
PurePursuit::PurePursuit(const ControlParameters& params)
    : LateralController(params),
      lpf_(std::make_shared<LowPassFilter>(params.pure_pursuit_lpf_alpha_,
                                           params.pure_pursuit_lpf_initial_value_)) {
                                            last_control_time_ = std::chrono::steady_clock::now();
                                           }

void PurePursuit::path_callback(const custom_interfaces::msg::PathPointArray& msg) {
  this->last_path_msg_ = msg.pathpoint_array;
  this->received_first_path_ = true;
}

void PurePursuit::vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) {
  this->last_velocity_msg_ = msg;
  this->absolute_velocity_ =
      std::sqrt(msg.velocity_x * msg.velocity_x + msg.velocity_y * msg.velocity_y);
  this->received_first_state_ = true;
}

void PurePursuit::vehicle_pose_callback(const custom_interfaces::msg::Pose& vehicle_pose_msg) {
  this->last_pose_msg_ = vehicle_pose_msg;
  this->received_first_pose_ = true;
}

double PurePursuit::get_steering_command() {
  double steering_command = 0.0;

  if (this->received_first_path_ && this->received_first_state_ && this->received_first_pose_) {
    // --- timing ---
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_control_time_).count();
    // protect against first-run / timer jitter
    if (dt <= 0.0) dt = 1e-3;
    if (dt > 0.2) dt = 0.2;  // optional clamp

    // --- Pure Pursuit nominal command (u_pp) ---
    Position vehicle_cog(this->last_pose_msg_.x, this->last_pose_msg_.y);
    Position rear_axis = ::rear_axis_position(vehicle_cog, this->last_pose_msg_.theta,
                                              this->params_->car_parameters_.dist_cg_2_rear_axis);

    auto [closest_point, closest_point_id, closest_point_velocity] =
        ::get_closest_point(this->last_path_msg_, rear_axis);

    double u_pp = 0.0;
    if (closest_point_id != -1) {
      const double ld =
          std::max(this->params_->pure_pursuit_lookahead_gain_ * this->absolute_velocity_,
                   this->params_->pure_pursuit_lookahead_minimum_);

      auto [lookahead_point, lookahead_velocity, lookahead_error] =
          ::get_lookahead_point(this->last_path_msg_, closest_point_id, ld, rear_axis,
                                this->params_->first_last_max_dist_);

      if (!lookahead_error) {
        u_pp = pp_steering_control_law(rear_axis, vehicle_cog, lookahead_point,
                                       this->params_->car_parameters_.dist_cg_2_rear_axis);
      }
    }

    // --- Lateral PID (distance + heading) ---
    double u_pid = 0.0;
    if (closest_point_id >= 2 &&
        closest_point_id + 1 < static_cast<int>(this->last_path_msg_.size())) {
      // neighbors
      Position prev_point(this->last_path_msg_[closest_point_id - 1].x,
                          this->last_path_msg_[closest_point_id - 1].y);
      Position next_point(this->last_path_msg_[closest_point_id + 1].x,
                          this->last_path_msg_[closest_point_id + 1].y);

      // segment vector
      const double vx = next_point.x - prev_point.x;
      const double vy = next_point.y - prev_point.y;

      // --- project REAR AXLE (not CoG) onto segment ---
      const double wx = rear_axis.x - prev_point.x;
      const double wy = rear_axis.y - prev_point.y;
      const double vv = vx * vx + vy * vy;
      double t = (vv > 1e-12) ? (wx * vx + wy * vy) / vv : 0.0;
      t = std::clamp(t, 0.0, 1.0);

      Position interp_closest{prev_point.x + t * vx, prev_point.y + t * vy};

      // heading of path segment
      const double segment_heading = std::atan2(vy, vx);

      // heading error (wrap to [-pi, pi])
      double heading_error = last_pose_msg_.theta - segment_heading;
      while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
      while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

      // signed lateral error of the REAR AXLE
      const double dx = rear_axis.x - interp_closest.x;
      const double dy = rear_axis.y - interp_closest.y;
      double distance_error = std::hypot(dx, dy);
      const double cross = vx * dy - vy * dx;  // +: left of path, -: right
      if (cross > 0.0) distance_error = -distance_error;

      // --- PID with derivative protection & back-calculated anti-windup ---
      u_pid = this->lateral_pid(
          distance_error, heading_error, dt,
          u_pp,  // feed-forward (PP) included in pre-sat sum
          this->params_->car_parameters_.steering_parameters.minimum_steering_angle,
          this->params_->car_parameters_.steering_parameters.maximum_steering_angle);
    }

    // Make the final command inside lateral_pid (which accounts for sat/antiwindup)
    steering_command = u_pid;  // u_pid already includes u_pp in the function below

    // timestamp for next cycle
    last_control_time_ = now;
  }

  // Final safety clamp
  steering_command = std::clamp(
      steering_command, this->params_->car_parameters_.steering_parameters.minimum_steering_angle,
      this->params_->car_parameters_.steering_parameters.maximum_steering_angle);

  return steering_command;
}

double PurePursuit::lateral_pid(double distance_error, double heading_error, double dt,
                                double u_pp,  // feed-forward (pure pursuit)
                                double u_min, double u_max) {
  // --- Derivatives with simple low-pass (T_d is a small time constant) ---

  RCLCPP_INFO(rclcpp::get_logger("control"),
                   "Lateral PID: dist_err=%.4f, head_err=%.4f, dt=%.4f, u_pp=%.4f, distance_p_gain=%.4f, distance_i_gain=%.4f, distance_d_gain=%.4f",
                   distance_error, heading_error, dt, u_pp, this->params_->distance_p_gain_, this->params_->distance_i_gain_, this->params_->distance_d_gain_);

  const double Td_dist = 0.05;  // s
  const double Td_head = 0.05;  // s
  const double alpha_d = Td_dist / (Td_dist + dt);
  const double alpha_h = Td_head / (Td_head + dt);

  const double d_dist_raw = (distance_error - previous_distance_error_) / dt;
  const double d_head_raw = (heading_error - previous_heading_error_) / dt;

  distance_d_filt_ = alpha_d * distance_d_filt_ + (1.0 - alpha_d) * d_dist_raw;
  heading_d_filt_ = alpha_h * heading_d_filt_ + (1.0 - alpha_h) * d_head_raw;

  previous_distance_error_ = distance_error;
  previous_heading_error_ = heading_error;

  // --- Form the unsaturated control (before anti-windup) ---
  // NOTE: Integrators are updated AFTER anti-windup term is computed.
  const double u_p_d = this->params_->distance_p_gain_ * distance_error;
  const double u_d_d = this->params_->distance_d_gain_ * distance_d_filt_;
  const double u_p_h = this->params_->heading_p_gain_ * heading_error;
  const double u_d_h = this->params_->heading_d_gain_ * heading_d_filt_;

  const double u_unsat = u_pp + u_p_d + this->params_->distance_i_gain_ * integral_distance_error_ + u_d_d +
                         u_p_h + this->params_->heading_i_gain_ * integral_heading_error_ + u_d_h;

  // --- Saturate and compute back-calculation signal ---
  const double u_sat = std::clamp(u_unsat, u_min, u_max);
  const double aw_err = u_sat - u_unsat;  // negative if we asked too much

  // --- Distribute anti-windup to both integrators proportional to their Ki ---
  const double Ki_d = std::abs(this->params_->distance_i_gain_);
  const double Ki_h = std::abs(this->params_->heading_i_gain_);
  const double Ki_sum = Ki_d + Ki_h;
  double w_d = 0.5, w_h = 0.5;  // default split
  if (Ki_sum > 1e-12) {
    w_d = Ki_d / Ki_sum;
    w_h = Ki_h / Ki_sum;
  }

  // kaw_ is your anti_windup_gain_ (tune ~ 1/Ki magnitude)
  integral_distance_error_ += (distance_error + kaw_ * w_d * aw_err) * dt;
  integral_heading_error_ += (heading_error + kaw_ * w_h * aw_err) * dt;

  // Optional hard clamp of integrators as a safety net (not the primary anti-windup)
  integral_distance_error_ =
      std::clamp(integral_distance_error_, - this->params_->distance_anti_windup_, this->params_->distance_anti_windup_);
  integral_heading_error_ =
      std::clamp(integral_heading_error_, -this->params_->heading_anti_windup_, this->params_->heading_anti_windup_);

  // Recompute the control using the updated integrators (still return the saturated value)
  const double u_pid = u_pp + u_p_d + this->params_->distance_i_gain_ * integral_distance_error_ + u_d_d + u_p_h +
                       this->params_->heading_i_gain_ * integral_heading_error_ + u_d_h;

  return std::clamp(u_pid, u_min, u_max);
}

double PurePursuit::pp_steering_control_law(Position rear_axis, Position cg,
                                            Position lookahead_point, double dist_cg_2_rear_axis) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);
  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);

  double steering_angle =
      std::atan(2 * this->params_->car_parameters_.wheelbase * std::sin(alpha) / ld);
  double filtered_steering_angle = lpf_->filter(steering_angle);

  return std::clamp(filtered_steering_angle,
                    this->params_->car_parameters_.steering_parameters.minimum_steering_angle,
                    this->params_->car_parameters_.steering_parameters.maximum_steering_angle);
}

double PurePursuit::calculate_alpha(Position vehicle_rear_wheel, Position vehicle_cg,
                                    Position lookahead_point, double dist_cg_2_rear_axis) {
  double lookhead_point_2_rear_wheel = vehicle_rear_wheel.euclidean_distance(lookahead_point);
  double lookhead_point_2_cg = lookahead_point.euclidean_distance(vehicle_cg);

  // Law of cosines
  double cos_alpha = (std::pow(lookhead_point_2_rear_wheel, 2) + std::pow(dist_cg_2_rear_axis, 2) -
                      std::pow(lookhead_point_2_cg, 2)) /
                     (2 * lookhead_point_2_rear_wheel * dist_cg_2_rear_axis);

  cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
  double alpha = std::acos(cos_alpha);

  if (cross_product(vehicle_rear_wheel, vehicle_cg, lookahead_point) < 0) {
    alpha = -alpha;
  }

  return alpha;
}