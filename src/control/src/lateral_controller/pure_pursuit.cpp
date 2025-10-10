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
  if (!this->received_first_path_ || !this->received_first_state_ || !this->received_first_pose_) {
    return 0;
  }

  // Prepare inputs for the Pure Pursuit control law
  Position vehicle_cog = Position(this->last_pose_msg_.x, this->last_pose_msg_.y);
  Position rear_axis = rear_axis_position(vehicle_cog, this->last_pose_msg_.theta,
      this->params_->car_parameters_.dist_cg_2_rear_axis);
  auto [closest_point, closest_point_id, closest_point_velocity] =
      get_closest_point(this->last_path_msg_, rear_axis);
  if (closest_point_id == -1) {
    return 0;            
  }
  double ld = std::max(this->params_->pure_pursuit_lookahead_gain_ * this->absolute_velocity_,
                       this->params_->pure_pursuit_lookahead_minimum_);
  auto [lookahead_point, lookahead_velocity, lookahead_error] =
      get_lookahead_point(this->last_path_msg_, closest_point_id, ld, rear_axis, this->params_->first_last_max_dist_);

  if (lookahead_error) {
    return 0;
  }

  return pp_steering_control_law(rear_axis, vehicle_cog,
      lookahead_point, this->params_->car_parameters_.dist_cg_2_rear_axis);
}

double PurePursuit::pp_steering_control_law(Position rear_axis, Position cg,
                                            Position lookahead_point, double dist_cg_2_rear_axis) {
  double alpha = calculate_alpha(rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);
  // update lookahead distance to the actual distance
  double ld = rear_axis.euclidean_distance(lookahead_point);

  double steering_angle = atan(2 * this->params_->car_parameters_.wheelbase * sin(alpha) / ld);
  double filtered_steering_angle = lpf_->filter(steering_angle);

  return std::clamp(filtered_steering_angle, this->params_->car_parameters_.steering_parameters.minimum_steering_angle,
    this->params_->car_parameters_.steering_parameters.maximum_steering_angle);
}

double PurePursuit::calculate_alpha(Position vehicle_rear_wheel, Position vehicle_cg,
                                    Position lookahead_point, double dist_cg_2_rear_axis) {
  double lookhead_point_2_rear_wheel = vehicle_rear_wheel.euclidean_distance(lookahead_point);
  double lookhead_point_2_cg = lookahead_point.euclidean_distance(vehicle_cg);

  // Law of cosines
  double cos_alpha = (pow(lookhead_point_2_rear_wheel, 2) + pow(dist_cg_2_rear_axis, 2) -
                      pow(lookhead_point_2_cg, 2)) /
                     (2 * lookhead_point_2_rear_wheel * dist_cg_2_rear_axis);

  cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
  double alpha = acos(cos_alpha);

  if (cross_product(vehicle_rear_wheel, vehicle_cg, lookahead_point) < 0) {
    alpha = -alpha;
  }

  return alpha;
}