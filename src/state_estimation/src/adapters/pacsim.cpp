#include "adapters/pacsim.hpp"

PacsimAdapter::PacsimAdapter(const SEParameters& parameters) : SENode(parameters) {
  // Sensor Subscriptions
  _imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/pacsim/imu/cog_imu", 10,
      std::bind(&PacsimAdapter::imu_callback, this, std::placeholders::_1));
  _wheel_speeds_sub_ = this->create_subscription<pacsim::msg::Wheels>(
      "/pacsim/wheelspeeds", 10,
      std::bind(&PacsimAdapter::wss_callback, this, std::placeholders::_1));
  _steering_angle_sub_ = this->create_subscription<pacsim::msg::StampedScalar>(
      "/pacsim/steeringFront", 10,
      std::bind(&PacsimAdapter::steering_angle_callback, this, std::placeholders::_1));

  // Control Subscriptions
  _throttle_command_sub_ = this->create_subscription<pacsim::msg::Wheels>(
      "/pacsim/throttle_setpoint", 10,
      std::bind(&PacsimAdapter::throttle_command_callback, this, std::placeholders::_1));
  _steering_command_sub_ = this->create_subscription<pacsim::msg::StampedScalar>(
      "/pacsim/steering_setpoint", 10,
      std::bind(&PacsimAdapter::steering_command_callback, this, std::placeholders::_1));
}

void PacsimAdapter::throttle_command_callback(const pacsim::msg::Wheels::SharedPtr msg) {
  this->_latest_control_command.throttle_fl = msg->fl;
  this->_latest_control_command.throttle_fr = msg->fr;
  this->_latest_control_command.throttle_rl = msg->rl;
  this->_latest_control_command.throttle_rr = msg->rr;

  if (!received_steering_command_) {
    received_throttle_command_ = true;
  } else {
    this->_state_estimator_->control_callback(_latest_control_command);
    received_steering_command_ = false;  // Reset for next command
    received_throttle_command_ = false;  // Reset for next command
  }
}

void PacsimAdapter::steering_command_callback(const pacsim::msg::StampedScalar::SharedPtr msg) {
  this->_latest_control_command.steering_angle = msg->value;

  if (!received_throttle_command_) {
    received_steering_command_ = true;
  } else {
    this->_state_estimator_->control_callback(_latest_control_command);
    received_steering_command_ = false;  // Reset for next command
    received_throttle_command_ = false;  // Reset for next command
  }
}

void PacsimAdapter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  common_lib::sensor_data::ImuData imu_data;
  imu_data.acceleration_x = msg->linear_acceleration.x;
  imu_data.acceleration_y = msg->linear_acceleration.y;
  imu_data.rotational_velocity = msg->angular_velocity.z;
  imu_data.timestamp_ = msg->header.stamp;

  this->_state_estimator_->imu_callback(imu_data);
}

void PacsimAdapter::wss_callback(const pacsim::msg::Wheels::SharedPtr msg) {
  common_lib::sensor_data::WheelEncoderData wheel_data;
  wheel_data.fl_rpm = msg->fl;
  wheel_data.fr_rpm = msg->fr;
  wheel_data.rl_rpm = msg->rl;
  wheel_data.rr_rpm = msg->rr;
  wheel_data.timestamp_ = msg->stamp;

  this->_state_estimator_->wss_callback(wheel_data);
}

void PacsimAdapter::steering_angle_callback(const pacsim::msg::StampedScalar::SharedPtr msg) {
  this->_state_estimator_->steering_callback(msg->value);
}