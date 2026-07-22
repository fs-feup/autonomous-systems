#include "adapters/invictasim_adapter.hpp"

InvictaSimAdapter::InvictaSimAdapter(const VEParameters& parameters) : VENode(parameters) {
  this->_free_acceleration_subscription_.subscribe(this, "/invictasim/imu/free_acceleration");
  this->_angular_velocity_subscription_.subscribe(this, "/invictasim/imu/angular_velocity");
  const XsensImuPolicy xsens_imu_policy(10);
  this->_xsens_imu_sync_ = std::make_shared<message_filters::Synchronizer<XsensImuPolicy>>(
      xsens_imu_policy, _free_acceleration_subscription_, _angular_velocity_subscription_);
  this->_xsens_imu_sync_->registerCallback(&InvictaSimAdapter::imu_callback, this);

  this->_fl_wheel_rpm_subscription_.subscribe(this, "/invictasim/wss/front_left");
  this->_fr_wheel_rpm_subscription_.subscribe(this, "/invictasim/wss/front_right");

  const WheelSSPolicy policy(10);
  this->_wss_sync_ = std::make_shared<message_filters::Synchronizer<WheelSSPolicy>>(
      policy, _fl_wheel_rpm_subscription_, _fr_wheel_rpm_subscription_);
  this->_wss_sync_->registerCallback(&InvictaSimAdapter::wss_callback, this);

  this->_steering_angle_sub_ = this->create_subscription<custom_interfaces::msg::SteeringAngle>(
      "/invictasim/steering_angle_sensor", 1,
      std::bind(&InvictaSimAdapter::steering_angle_callback, this, std::placeholders::_1));
  this->_resolver_sub_ = this->create_subscription<custom_interfaces::msg::WheelRPM>(
      "/invictasim/resolver", 1,
      std::bind(&InvictaSimAdapter::resolver_callback, this, std::placeholders::_1));
}

void InvictaSimAdapter::wss_callback(const custom_interfaces::msg::WheelRPM& fl_wheel_rpm_msg,
                                     const custom_interfaces::msg::WheelRPM& fr_wheel_rpm_msg) {
  common_lib::sensor_data::WheelEncoderData wss_data;
  wss_data.rl_rpm = 0;
  wss_data.rr_rpm = 0;
  wss_data.fl_rpm = fl_wheel_rpm_msg.fl_rpm;
  wss_data.fr_rpm = fr_wheel_rpm_msg.fr_rpm;
  this->_velocity_estimator_->wss_callback(wss_data);
  this->publish_velocities();
}

void InvictaSimAdapter::steering_angle_callback(const custom_interfaces::msg::SteeringAngle msg) {
  this->_velocity_estimator_->steering_callback(msg.steering_angle);
  this->publish_velocities();
}

void InvictaSimAdapter::imu_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& free_acceleration_msg,
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& angular_velocity_msg) {
  common_lib::sensor_data::ImuData imu_data;
  imu_data.rotational_velocity = angular_velocity_msg->vector.z;
  imu_data.acceleration_x = free_acceleration_msg->vector.x;
  imu_data.acceleration_y = free_acceleration_msg->vector.y;
  this->_velocity_estimator_->imu_callback(imu_data);
  this->publish_velocities();
}

void InvictaSimAdapter::resolver_callback(custom_interfaces::msg::WheelRPM msg) {
  this->_velocity_estimator_->motor_rpm_callback(msg.rr_rpm);
  this->publish_velocities();
}