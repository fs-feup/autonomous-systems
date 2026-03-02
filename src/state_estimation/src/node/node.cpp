#include "node/node.hpp"

SENode::SENode(const std::shared_ptr<SEParameters>& parameters)
    : Node("state_estimation"), _params_(parameters) {
  if (this->_params_->adapter_ == "pacsim") {
    this->_params_->observation_model_name_ = "pacsim_sensors";
  }

  this->_state_estimator_ = estimators_map.at(this->_params_->estimation_method_)(
      this->_params_, process_models_map.at(this->_params_->process_model_name_)(this->_params_),
      observation_models_map.at(this->_params_->observation_model_name_)(this->_params_));

  // Timer Subscription if needed
  if (this->_params_->state_estimation_freq_ > 0) {
    auto timer_period =
        std::chrono::milliseconds(static_cast<int>(1000 / this->_params_->state_estimation_freq_));
    this->_timer_ = this->create_wall_timer(timer_period, [this]() { this->timer_callback(); });
  }

  // Publishers
  this->_execution_time_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/state_estimation/execution_time", 10);

  this->_state_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
      "/state_estimation/vehicle_state", 10);

  this->_velocity_pub_ = this->create_publisher<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", 10);
}

void SENode::publish_state(const State& state, const rclcpp::Time time) {
  custom_interfaces::msg::VehicleStateVector state_msg;
  state_msg.header.stamp = time;
  state_msg.velocity_x = state(VX);
  state_msg.velocity_y = state(VY);
  state_msg.yaw_rate = state(YAW_RATE);
  state_msg.acceleration_x = state(AX);
  state_msg.acceleration_y = state(AY);
  state_msg.steering_angle = state(ST_ANGLE);
  state_msg.fl_rpm = state(FL_WHEEL_SPEED);
  state_msg.fr_rpm = state(FR_WHEEL_SPEED);
  state_msg.rl_rpm = state(RL_WHEEL_SPEED);
  state_msg.rr_rpm = state(RR_WHEEL_SPEED);
  this->_state_pub_->publish(state_msg);

  custom_interfaces::msg::Velocities velocity_msg;
  velocity_msg.header.stamp = time;
  velocity_msg.velocity_x = state(VX);
  velocity_msg.velocity_y = state(VY);
  velocity_msg.angular_velocity = state(YAW_RATE);
  this->_velocity_pub_->publish(velocity_msg);
}

void SENode::timer_callback() {
  rclcpp::Time start_time = this->get_clock()->now();
  State curr_state;
  this->_state_estimator_->timer_callback(curr_state);
  publish_state(curr_state, start_time);
  rclcpp::Time end_time = this->get_clock()->now();
  std_msgs::msg::Float64 execution_time_msg;
  execution_time_msg.data = (end_time - start_time).seconds() * 1000;
  this->_execution_time_pub_->publish(execution_time_msg);
}