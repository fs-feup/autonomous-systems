#include "adapters/vehicle.hpp"

VehicleAdapter::VehicleAdapter(const std::shared_ptr<SEParameters>& parameters)
    : SENode(parameters) {
  // IMU: free acceleration and angular velocity are published separately and synchronized.
  this->_free_acceleration_subscription_.subscribe(this, "/filter/free_acceleration");
  this->_angular_velocity_subscription_.subscribe(this, "/imu/angular_velocity");
  const XsensImuPolicy xsens_imu_policy(10);
  this->_xsens_imu_sync_ = std::make_shared<message_filters::Synchronizer<XsensImuPolicy>>(
      xsens_imu_policy, _free_acceleration_subscription_, _angular_velocity_subscription_);
  this->_xsens_imu_sync_->registerCallback(&VehicleAdapter::imu_callback, this);

  // Front wheel-speed sensors (no rear wheel-speed sensors on this car).
  this->_fl_wheel_rpm_subscription_.subscribe(this, "/vehicle/fl_rpm");
  this->_fr_wheel_rpm_subscription_.subscribe(this, "/vehicle/fr_rpm");
  const WheelSSPolicy policy(10);
  this->_wss_sync_ = std::make_shared<message_filters::Synchronizer<WheelSSPolicy>>(
      policy, _fl_wheel_rpm_subscription_, _fr_wheel_rpm_subscription_);
  this->_wss_sync_->registerCallback(&VehicleAdapter::wss_callback, this);

  this->_steering_angle_sub_ = this->create_subscription<custom_interfaces::msg::SteeringAngle>(
      "/vehicle/bosch_steering_angle", 1,
      std::bind(&VehicleAdapter::steering_angle_callback, this, std::placeholders::_1));
  this->_resolver_sub_ = this->create_subscription<custom_interfaces::msg::WheelRPM>(
      "/vehicle/motor_rpm", 1,
      std::bind(&VehicleAdapter::resolver_callback, this, std::placeholders::_1));

  // Control output from the control package, used by the process model's prediction.
  this->_control_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/control/command", 10,
      std::bind(&VehicleAdapter::control_callback, this, std::placeholders::_1));
}

void VehicleAdapter::imu_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& free_acceleration_msg,
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& angular_velocity_msg) {
  // TODO: treat IMU data, correct bias + frame fixes and calibration
  // Estimate the yaw-rate bias from the first readings (car stationary) before forwarding data.
  if (this->number_of_imu_readings_ < 250) {
    this->average_imu_bias_ =
        (this->average_imu_bias_ * this->number_of_imu_readings_ + angular_velocity_msg->vector.z) /
        (this->number_of_imu_readings_ + 1);
    this->number_of_imu_readings_++;
    return;
  }

  common_lib::sensor_data::ImuData imu_data;
  imu_data.acceleration_x = free_acceleration_msg->vector.x;
  imu_data.acceleration_y = free_acceleration_msg->vector.y;
  imu_data.rotational_velocity = angular_velocity_msg->vector.z - this->average_imu_bias_;
  imu_data.timestamp_ = free_acceleration_msg->header.stamp;

  this->_state_estimator_->imu_callback(imu_data);
}

void VehicleAdapter::wss_callback(const custom_interfaces::msg::WheelRPM& fl_wheel_rpm_msg,
                                  const custom_interfaces::msg::WheelRPM& fr_wheel_rpm_msg) {
  // Only front wheel-speed sensors exist
  common_lib::sensor_data::WheelEncoderData wss_data;
  wss_data.fl_rpm = fl_wheel_rpm_msg.fl_rpm;
  wss_data.fr_rpm = fr_wheel_rpm_msg.fr_rpm;
  wss_data.rl_rpm = 0.0;
  wss_data.rr_rpm = 0.0;
  wss_data.timestamp_ = fl_wheel_rpm_msg.header.stamp;

  this->_state_estimator_->wss_callback(wss_data);
}

void VehicleAdapter::steering_angle_callback(const custom_interfaces::msg::SteeringAngle msg) {
  this->_state_estimator_->steering_callback(msg.steering_angle, rclcpp::Time(msg.header.stamp));
}

void VehicleAdapter::resolver_callback(custom_interfaces::msg::WheelRPM msg) {
  // The resolver publishes the motor RPM in the rr_rpm field of a WheelRPM message.
  this->_state_estimator_->motor_rpm_callback(msg.rr_rpm, rclcpp::Time(msg.header.stamp));
}

void VehicleAdapter::control_callback(const custom_interfaces::msg::ControlCommand msg) {
  common_lib::structures::ControlCommand control_command;
  control_command.throttle_fl = msg.throttle_fl;
  control_command.throttle_fr = msg.throttle_fr;
  control_command.throttle_rl = msg.throttle_rl;
  control_command.throttle_rr = msg.throttle_rr;
  control_command.steering_angle = msg.steering;
  this->_state_estimator_->control_callback(control_command);
}
