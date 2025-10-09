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
  common_lib::sensor_data::ImuData imu_data(msg->angular_velocity.z, msg->linear_acceleration.x,
                                            msg->linear_acceleration.y, msg->header.stamp);
  this->_velocity_estimator_->imu_callback(imu_data);
  // this->publish_velocities();
}

void PacsimAdapter::wss_callback(const pacsim::msg::Wheels::SharedPtr msg) {
  common_lib::sensor_data::WheelEncoderData wheel_encoder_data(msg->rl, msg->rr, msg->fl, msg->fr,
                                                               0.0, msg->stamp);
  this->_velocity_estimator_->motor_rpm_callback(0.5 *
                                                 this->_parameters_.car_parameters_.gear_ratio *
                                                 (msg->rl + msg->rr));  // Simulate resolver data
  this->_velocity_estimator_->wss_callback(wheel_encoder_data);
  this->publish_velocities();
}

void PacsimAdapter::steering_angle_callback(const pacsim::msg::StampedScalar::SharedPtr msg) {
  this->_velocity_estimator_->steering_callback(msg->value);
  // this->publish_velocities();
}