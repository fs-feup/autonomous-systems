#include "adapters/pacsim_adapter.hpp"

PacsimAdapter::PacsimAdapter(const VEParameters& parameters) : VENode(parameters) {
  _imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/pacsim/imu/cog_imu", 1,
      std::bind(&PacsimAdapter::ImuCallback, this, std::placeholders::_1));
  wheel_speeds_sub_ = this->create_subscription<pacsim::msg::Wheels>(
      "/pacsim/wheelspeeds", 1,
      std::bind(&PacsimAdapter::WheelSpeedsCallback, this, std::placeholders::_1));
  _steering_angle_subscription_ = this->create_subscription<pacsim::msg::StampedScalar>(
      "/pacsim/steeringFront", 1,
      std::bind(&PacsimAdapter::SteeringAngleCallback, this, std::placeholders::_1));
}

void PacsimAdapter::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  common_lib::sensor_data::ImuData imu_data;
  imu_data.acceleration_x = msg->linear_acceleration.x;
  imu_data.acceleration_y = msg->linear_acceleration.y;
  imu_data.rotational_velocity = msg->angular_velocity.z;
  this->_velocity_estimator_->IMUCallback(imu_data);
}

void PacsimAdapter::WheelSpeedsCallback(const pacsim::msg::Wheels::SharedPtr msg) {
  common_lib::sensor_data::WheelEncoderData wheel_encoder_data;
  wheel_encoder_data.fl_rpm = msg->fl;
  wheel_encoder_data.fr_rpm = msg->fr;
  wheel_encoder_data.rl_rpm = msg->rl;
  wheel_encoder_data.rr_rpm = msg->rr;
  this->_velocity_estimator_->WSSCallback(wheel_encoder_data);
  this->_velocity_estimator_->ResolverCallback(
      2 * (msg->rl + msg->rr));  // Simulate resolver data assuming gear ratio of 4:1
}

void PacsimAdapter::SteeringAngleCallback(const pacsim::msg::StampedScalar::SharedPtr msg) {
  this->_velocity_estimator_->SteeringCallback(msg->value);
}