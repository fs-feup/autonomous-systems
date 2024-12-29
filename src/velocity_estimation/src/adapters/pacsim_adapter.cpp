#include "adapters/pacsim_adapter.hpp"

PacsimAdapter::PacsimAdapter(const VEParameters& parameters) : VENode(parameters) {
  _imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/pacsim/imu/cog_imu", 1,
      std::bind(&PacsimAdapter::imu_callback, this, std::placeholders::_1));
  wheel_speeds_sub_ = this->create_subscription<pacsim::msg::Wheels>(
      "/pacsim/wheelspeeds", 1,
      std::bind(&PacsimAdapter::wss_callback, this, std::placeholders::_1));
  _steering_angle_sub_ = this->create_subscription<pacsim::msg::StampedScalar>(
      "/pacsim/steeringFront", 1,
      std::bind(&PacsimAdapter::steering_angle_callback, this, std::placeholders::_1));
}

void PacsimAdapter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  common_lib::sensor_data::ImuData imu_data;
  imu_data.acceleration_x = msg->linear_acceleration.x;
  imu_data.acceleration_y = msg->linear_acceleration.y;
  imu_data.rotational_velocity = msg->angular_velocity.z;
  this->_velocity_estimator_->imu_callback(imu_data);
}

void PacsimAdapter::wss_callback(const pacsim::msg::Wheels::SharedPtr msg) {
  common_lib::sensor_data::WheelEncoderData wheel_encoder_data;
  wheel_encoder_data.fl_rpm = msg->fl;
  wheel_encoder_data.fr_rpm = msg->fr;
  wheel_encoder_data.rl_rpm = msg->rl;
  wheel_encoder_data.rr_rpm = msg->rr;
  this->_velocity_estimator_->wss_callback(wheel_encoder_data);
  this->_velocity_estimator_->motor_rpm_callback(
      2 * (msg->rl + msg->rr));  // Simulate resolver data assuming gear ratio of 4:1
}

void PacsimAdapter::steering_angle_callback(const pacsim::msg::StampedScalar::SharedPtr msg) {
  this->_velocity_estimator_->steering_callback(msg->value);
}