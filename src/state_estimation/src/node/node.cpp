#include <chrono>

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
  this->_execution_time_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/state_estimation/execution_time", 10);

  this->_state_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
      "/state_estimation/vehicle_state", 10);

  this->_velocity_pub_ = this->create_publisher<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", 10);

  this->_slip_ratio_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "/state_estimation/slip_ratio", 10);
  this->_slip_angles_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "/state_estimation/slip_angles", 10);
  this->_tire_forces_pub_ = this->create_publisher<custom_interfaces::msg::TireForces>(
      "/state_estimation/tire_forces", 10);
  this->_sensor_health_pub_ = this->create_publisher<custom_interfaces::msg::SensorHealth>(
      "/state_estimation/sensor_health", 10);
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
  state_msg.fl_rpm = state(FL_WHEEL_SPEED) * (60 / (2 * M_PI));
  state_msg.fr_rpm = state(FR_WHEEL_SPEED) * (60 / (2 * M_PI));
  state_msg.rl_rpm = state(RL_WHEEL_SPEED) * (60 / (2 * M_PI));
  state_msg.rr_rpm = state(RR_WHEEL_SPEED) * (60 / (2 * M_PI));

  custom_interfaces::msg::Velocities velocity_msg;
  velocity_msg.header.stamp = time;
  velocity_msg.velocity_x = state(VX);
  velocity_msg.velocity_y = state(VY);
  velocity_msg.angular_velocity = state(YAW_RATE);
  // Slam does not use it i am sure like 85% but put it because i am a scared man and its 2 AM
  velocity_msg.covariance[0] = 1e-4;
  velocity_msg.covariance[4] = 1e-4;
  velocity_msg.covariance[8] = 1e-4;

  this->_state_pub_->publish(state_msg);
  this->_velocity_pub_->publish(velocity_msg);
}

void SENode::timer_callback() {
  rclcpp::Time start_time = this->get_clock()->now();
  auto hrc_start = std::chrono::high_resolution_clock::now();
  State curr_state;
  this->_state_estimator_->timer_callback(curr_state);
  auto hrc_end = std::chrono::high_resolution_clock::now();
  publish_state(curr_state, start_time);

  // Publish execution times if enabled
  std_msgs::msg::Float64MultiArray execution_time_msg;

  if (this->_params_->publish_exec_times_) {
    Eigen::Vector4d execution_times = this->_state_estimator_->get_exec_times();
    // Array: [total_time, overhead, prediction, correction, update]
    execution_time_msg.data.resize(5);
    execution_time_msg.data[0] =
        std::chrono::duration<double, std::milli>(hrc_end - hrc_start).count();
    execution_time_msg.data[1] = execution_times(0);  // Overhead/input gathering (ms)
    execution_time_msg.data[2] = execution_times(1);  // Prediction stage (ms)
    execution_time_msg.data[3] = execution_times(2);  // Correction stage (ms)
    execution_time_msg.data[4] = execution_times(3);  // Update stage (ms)
  } else {
    execution_time_msg.data.resize(1);
    execution_time_msg.data[0] =
        std::chrono::duration<double, std::milli>(hrc_end - hrc_start).count();
  }
  this->_execution_time_pub_->publish(execution_time_msg);

  // Publish per-sensor health (Overseer). Generic over the sensor set: the names
  // travel in the message, so adding/removing sensors needs no change here.
  if (this->_params_->publish_sensor_health_) {
    const std::vector<SensorHealth> health = this->_state_estimator_->get_sensor_health();
    custom_interfaces::msg::SensorHealth health_msg;
    health_msg.header.stamp = start_time;
    health_msg.names.reserve(health.size());
    health_msg.statuses.reserve(health.size());
    for (const SensorHealth& sensor : health) {
      health_msg.names.push_back(sensor.name);
      health_msg.statuses.push_back(static_cast<uint8_t>(sensor.status));
    }
    this->_sensor_health_pub_->publish(health_msg);
  }

  if (!this->_params_->publish_vm_debug_info_) {
    return;  // Don't publish debug info if the parameter is set to false
  }

  VehicleState process_model_data = this->_state_estimator_->get_process_model_data();
  custom_interfaces::msg::WheelScalars slip_ratio_msg;
  slip_ratio_msg.header.stamp = start_time;
  slip_ratio_msg.fl = process_model_data.wheels_slip_ratio.front_left;
  slip_ratio_msg.fr = process_model_data.wheels_slip_ratio.front_right;
  slip_ratio_msg.rl = process_model_data.wheels_slip_ratio.rear_left;
  slip_ratio_msg.rr = process_model_data.wheels_slip_ratio.rear_right;
  this->_slip_ratio_pub_->publish(slip_ratio_msg);

  custom_interfaces::msg::WheelScalars slip_angles_msg;
  slip_angles_msg.header.stamp = start_time;
  slip_angles_msg.fl = process_model_data.wheels_slip_angle.front_left;
  slip_angles_msg.fr = process_model_data.wheels_slip_angle.front_right;
  slip_angles_msg.rl = process_model_data.wheels_slip_angle.rear_left;
  slip_angles_msg.rr = process_model_data.wheels_slip_angle.rear_right;
  this->_slip_angles_pub_->publish(slip_angles_msg);

  custom_interfaces::msg::TireForces tire_forces_msg;
  tire_forces_msg.header.stamp = start_time;

  // FL
  tire_forces_msg.fl_wrench.force.x = process_model_data.front_left_forces[0];
  tire_forces_msg.fl_wrench.force.y = process_model_data.front_left_forces[1];
  tire_forces_msg.fl_wrench.torque.y = process_model_data.front_left_forces[2];
  tire_forces_msg.fl_wrench.torque.z = process_model_data.front_left_forces[3];

  // FR
  tire_forces_msg.fr_wrench.force.x = process_model_data.front_right_forces[0];
  tire_forces_msg.fr_wrench.force.y = process_model_data.front_right_forces[1];
  tire_forces_msg.fr_wrench.torque.y = process_model_data.front_right_forces[2];
  tire_forces_msg.fr_wrench.torque.z = process_model_data.front_right_forces[3];

  // RL
  tire_forces_msg.rl_wrench.force.x = process_model_data.rear_left_forces[0];
  tire_forces_msg.rl_wrench.force.y = process_model_data.rear_left_forces[1];
  tire_forces_msg.rl_wrench.torque.y = process_model_data.rear_left_forces[2];
  tire_forces_msg.rl_wrench.torque.z = process_model_data.rear_left_forces[3];

  // RR
  tire_forces_msg.rr_wrench.force.x = process_model_data.rear_right_forces[0];
  tire_forces_msg.rr_wrench.force.y = process_model_data.rear_right_forces[1];
  tire_forces_msg.rr_wrench.torque.y = process_model_data.rear_right_forces[2];
  tire_forces_msg.rr_wrench.torque.z = process_model_data.rear_right_forces[3];

  this->_tire_forces_pub_->publish(tire_forces_msg);
}