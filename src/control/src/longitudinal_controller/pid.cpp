#include "longitudinal_controller/pid.hpp"

#include <cmath>

using namespace common_lib::structures;

/**
 * @brief Construct a new PID object
 *
 */
PID::PID(const ControlParameters& params) : LongitudinalController(params) {}

/**
 * @brief Calculate the output value
 *
 * @param setpoint
 * @param measurement
 * @return double
 */

double PID::update(double setpoint, double measurement) {
  double dt = static_cast<double>(this->params_->command_time_interval_) / 1000.0;
  const auto now = std::chrono::steady_clock::now();

  if (this->has_last_update_time_) {
    dt = std::chrono::duration<double>(now - this->last_update_time_).count();
    if (dt <= 0.0) {
      dt = static_cast<double>(this->params_->command_time_interval_) / 1000.0;
    }
  }

  this->last_update_time_ = now;
  this->has_last_update_time_ = true;

  /*
   * Error signal
   */
  double error = calculate_error(setpoint, measurement);

  this->calculate_proportional_term(error);

  this->calculate_integral_term(error, dt);

  /*
   * Derivative term , derivative on measurement
   */
  this->calculate_derivative_term(measurement, dt);

  /*
   * Anti-wind-up integrator
   */
  this->anti_wind_up();

  /*
   * Compute output and apply limits
   */
  this->compute_output();

  /*
   * Store error and measurement for the next iteration
   */
  this->prev_error_ = error;
  this->prev_measurement_ = measurement;

  /*
   * return output value
   */
  return this->out_;
}

double PID::calculate_error(double setpoint, double measurement) const {
  double error = setpoint - measurement;
  if (error > this->params_->pid_max_positive_error_) {
    error = this->params_->pid_max_positive_error_;
  } else if (error < this->params_->pid_max_negative_error_) {
    error = this->params_->pid_max_negative_error_;
  }else {
    // do nothing, error within limits
  }
  return error;
}

void PID::calculate_proportional_term(double error) {
  this->proportional_ = this->params_->pid_kp_ * error;
}

void PID::calculate_integral_term(double error, double dt) {
  const double sample_time = dt > 0.0 ? dt : 0.01;
  this->integrator_ = this->integrator_ + this->params_->pid_ki_ * error * sample_time;
}

void PID::anti_wind_up() {
  double curr_output = this->proportional_ + this->integrator_ + this->differentiator_;
  if (curr_output > this->params_->pid_lim_max_ || curr_output < this->params_->pid_lim_min_) {
    this->integrator_ = this->integrator_ * this->params_->pid_anti_windup_;
  }
}

void PID::calculate_derivative_term(double measurement, double dt) {
  const double sample_time = dt > 0.0 ? dt : 0.01;
  this->differentiator_ = -this->params_->pid_kd_ * (measurement - this->prev_measurement_) / sample_time;
}

void PID::compute_output() {
  this->out_ = this->proportional_ + this->integrator_ + this->differentiator_;

  if (this->out_ > this->params_->pid_lim_max_) {
    this->out_ = this->params_->pid_lim_max_;
  } else if (this->out_ < this->params_->pid_lim_min_) {
    this->out_ = this->params_->pid_lim_min_;
  } else {
    // do nothing, output within limits
  }
}

void PID::path_callback(const custom_interfaces::msg::PathPointArray& msg)  {
  this->last_path_msg_ = msg.pathpoint_array;
  this->received_first_path_ = true;
}
void PID::vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& msg)  {
  this->last_velocity_msg_ = msg;
  this->absolute_velocity_ = copysign(std::sqrt(msg.velocity_x * msg.velocity_x + msg.velocity_y * msg.velocity_y), msg.velocity_x);
  this->received_first_state_ = true;
}
void PID::vehicle_pose_callback(const custom_interfaces::msg::Pose& msg)  {
  this->last_pose_msg_ = msg;
  this->received_first_pose_ = true;
}

common_lib::structures::ControlCommand PID::get_throttle_command()  {
  common_lib::structures::ControlCommand command;
  
  if (this->received_first_path_ && this->received_first_state_ && this->received_first_pose_) {
    Position vehicle_cog = Position(this->last_pose_msg_.x, this->last_pose_msg_.y);
    Position rear_axis = ::rear_axis_position(vehicle_cog, this->last_pose_msg_.theta,
        this->params_->car_parameters_.cg_2_rear_axis);

    auto [closest_point, closest_point_id, closest_point_velocity] =
        ::get_closest_point(this->last_path_msg_, rear_axis);

    if (closest_point_id != -1) {
      command.throttle_rl = command.throttle_rr = update(closest_point_velocity, this->absolute_velocity_);
    }
  }
  
  return command;
}

void PID::publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  if (publisher_map.find("/pid/components") == publisher_map.end()) {
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/pid/components", 10);
    publisher_map["/pid/components"] = publisher;
  }

  auto publisher = std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>(publisher_map["/pid/components"]);
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {this->proportional_, this->integrator_, this->differentiator_};
  publisher->publish(msg);
}