#include "adapters/vehicle_adapter.hpp"

VehicleAdapter::VehicleAdapter(const VEParameters& parameters) : VENode(parameters) {
  this->_free_acceleration_subscription_.subscribe(this, "/filter/free_acceleration");
  this->_angular_velocity_subscription_.subscribe(this, "/imu/angular_velocity");
  const XsensImuPolicy xsens_imu_policy(10);
  this->_xsens_imu_sync_ = std::make_shared<message_filters::Synchronizer<XsensImuPolicy>>(
      xsens_imu_policy, _free_acceleration_subscription_, _angular_velocity_subscription_);
  this->_xsens_imu_sync_->registerCallback(&VehicleAdapter::imu_callback, this);

  this->_rl_wheel_rpm_subscription_.subscribe(this, "/vehicle/rl_rpm");
  this->_rr_wheel_rpm_subscription_.subscribe(this, "/vehicle/rr_rpm");

  const WheelSSPolicy policy(10);
  this->_wss_sync_ = std::make_shared<message_filters::Synchronizer<WheelSSPolicy>>(
      policy, _rl_wheel_rpm_subscription_, _rr_wheel_rpm_subscription_);
  this->_wss_sync_->registerCallback(&VehicleAdapter::wss_callback, this);

  this->_steering_angle_sub_ = this->create_subscription<custom_interfaces::msg::SteeringAngle>(
      "/vehicle/bosch_steering_angle", 1,
      std::bind(&VehicleAdapter::steering_angle_callback, this, std::placeholders::_1));
}

void VehicleAdapter::wss_callback(const custom_interfaces::msg::WheelRPM& rl_wheel_rpm_msg,
                                  const custom_interfaces::msg::WheelRPM& rr_wheel_rpm_msg) {
  common_lib::sensor_data::WheelEncoderData wss_data;
  wss_data.rl_rpm = rl_wheel_rpm_msg.rl_rpm;
  wss_data.rr_rpm = rr_wheel_rpm_msg.rr_rpm;
  wss_data.fl_rpm = rl_wheel_rpm_msg.rl_rpm;
  wss_data.fr_rpm = rr_wheel_rpm_msg.rr_rpm;
  this->_velocity_estimator_->wss_callback(wss_data);
  this->_velocity_estimator_->motor_rpm_callback(
      (rl_wheel_rpm_msg.rl_rpm + rr_wheel_rpm_msg.rr_rpm) * 2.0);
}

void VehicleAdapter::steering_angle_callback(const custom_interfaces::msg::SteeringAngle msg) {
  this->_velocity_estimator_->steering_callback(msg.steering_angle);
}

void VehicleAdapter::imu_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& free_acceleration_msg,
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& angular_velocity_msg) {
  common_lib::sensor_data::ImuData imu_data;
  imu_data.rotational_velocity = angular_velocity_msg->vector.z;
  imu_data.acceleration_x = free_acceleration_msg->vector.x;
  imu_data.acceleration_y = free_acceleration_msg->vector.y;
  this->_velocity_estimator_->imu_callback(imu_data);
  auto state = this->_velocity_estimator_->get_velocities();
  custom_interfaces::msg::Velocities velocities_msg;
  velocities_msg.header.stamp = rclcpp::Clock().now();
  velocities_msg.velocity_x = state.velocity_x;
  velocities_msg.velocity_y = state.velocity_y;
  velocities_msg.angular_velocity = state.rotational_velocity;
  this->_velocities_pub_->publish(velocities_msg);
}
