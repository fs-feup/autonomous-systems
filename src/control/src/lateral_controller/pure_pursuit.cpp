#include "lateral_controller/pure_pursuit.hpp"

using namespace common_lib::structures;

/**
 * @brief Pure Pursuit class Constructor
 *
 */
PurePursuit::PurePursuit(const ControlParameters& params) : LateralController(params),
    lpf_(std::make_shared<LowPassFilter>(params.pure_pursuit_lpf_alpha_, params.pure_pursuit_lpf_initial_value_)) {}

void PurePursuit::path_callback(const custom_interfaces::msg::PathPointArray& msg) {
  this->last_path_msg_ = msg.pathpoint_array;
  this->received_first_path_ = true;
}

void PurePursuit::vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) {
  this->last_velocity_msg_ = msg;
  this->absolute_velocity_ = std::sqrt(msg.velocity_x * msg.velocity_x + msg.velocity_y * msg.velocity_y);
  this->received_first_state_ = true;
}

void PurePursuit::vehicle_pose_callback(const custom_interfaces::msg::Pose& vehicle_pose_msg) {
  this->last_pose_msg_ = vehicle_pose_msg;
  this->received_first_pose_ = true;
}

double PurePursuit::get_steering_command()  {
  double steering_command = 0.0;
  
  if (this->received_first_path_ && this->received_first_state_ && this->received_first_pose_) {
    // Prepare inputs for the Pure Pursuit control law
    Position vehicle_cog = Position(this->last_pose_msg_.x, this->last_pose_msg_.y);
    Position rear_axis = ::rear_axis_position(vehicle_cog, this->last_pose_msg_.theta,
        this->params_->car_parameters_.dist_cg_2_rear_axis);
    auto [closest_point, closest_point_id, closest_point_velocity] =
        ::get_closest_point(this->last_path_msg_, rear_axis);
    
    if (closest_point_id != -1) {
      double ld = std::max(this->params_->pure_pursuit_lookahead_gain_ * this->absolute_velocity_,
                           this->params_->pure_pursuit_lookahead_minimum_);
      auto [lookahead_point, lookahead_velocity, lookahead_error] =
          ::get_lookahead_point(this->last_path_msg_, closest_point_id, ld, rear_axis, this->params_->first_last_max_dist_);

      if (!lookahead_error) {
        steering_command = pp_steering_control_law(rear_axis, vehicle_cog,
            lookahead_point, this->params_->car_parameters_.dist_cg_2_rear_axis);
      }
    }

    // Get the actual closest point (interpolation)
    if (closest_point_id >= 2 && closest_point_id + 1 < this->last_path_msg_.size()) {
      Position prev_point(this->last_path_msg_[closest_point_id - 1].x,
                          this->last_path_msg_[closest_point_id - 1].y);
      Position next_point(this->last_path_msg_[closest_point_id + 1].x,
                          this->last_path_msg_[closest_point_id + 1].y);

      // --- Step 1: Project rear_axis onto the segment (prev_point, next_point)
      double vx = next_point.x - prev_point.x;
      double vy = next_point.y - prev_point.y;
      double wx = last_pose_msg_.x - prev_point.x;
      double wy = last_pose_msg_.y - prev_point.y;

      double vv = vx * vx + vy * vy;
      double t = 0.0;
      if (vv > 1e-9) {
          t = (wx * vx + wy * vy) / vv;
      }

      // Clamp to the segment
      t = std::max(0.0, std::min(1.0, t));

      // Interpolated (projected) closest point
      Position interpolated_closest_point;
      interpolated_closest_point.x = prev_point.x + t * vx;
      interpolated_closest_point.y = prev_point.y + t * vy;

      // --- Step 2: Compute heading of the line segment
      double segment_heading = std::atan2(vy, vx);

      // --- Step 3: Compute heading error (normalize between -pi and pi)
      double heading_error = last_pose_msg_.theta - segment_heading;
      while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
      while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

      double dx = last_pose_msg_.x - interpolated_closest_point.x;
      double dy = last_pose_msg_.y - interpolated_closest_point.y;

      // The sign comes from the cross product (segment vector × vehicle vector)
      // If positive -> vehicle is to the LEFT of the path direction
      // If negative -> vehicle is to the RIGHT
      double cross = vx * dy - vy * dx;  // determinant in 2D
      double distance_error = std::sqrt(dx * dx + dy * dy);
      if (cross > 0) distance_error = -distance_error;  // assign side

      const double distance_error_multiplier = 0.125;
      const double heading_error_multiplier = 0.0;

      steering_command += distance_error_multiplier * distance_error  + heading_error_multiplier * heading_error;

    }

  }
  
  return steering_command;
}

double PurePursuit::pp_steering_control_law(Position rear_axis, Position cg,
                                            Position lookahead_point, double dist_cg_2_rear_axis) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);
  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);

  double steering_angle = std::atan(2 * this->params_->car_parameters_.wheelbase * std::sin(alpha) / ld);
  double filtered_steering_angle = lpf_->filter(steering_angle);

  return std::clamp(filtered_steering_angle, this->params_->car_parameters_.steering_parameters.minimum_steering_angle,
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