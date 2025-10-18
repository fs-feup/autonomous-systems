#include "adapters/vehicle_adapter.hpp"

VehicleAdapter::VehicleAdapter(const VEParameters& parameters) : VENode(parameters) {
  this->_angular_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/angular_velocity", 1,
      std::bind(&VehicleAdapter::angular_velocity_callback, this, std::placeholders::_1));
  this->_free_acceleration_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/filter/free_acceleration", 1,
      std::bind(&VehicleAdapter::free_acceleration_callback, this, std::placeholders::_1));

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
}

void VehicleAdapter::wss_callback(const custom_interfaces::msg::WheelRPM& fl_wheel_rpm_msg,
                                  const custom_interfaces::msg::WheelRPM& fr_wheel_rpm_msg) {
  common_lib::sensor_data::WheelEncoderData wss_data;
  wss_data.rl_rpm = 0;
  wss_data.rr_rpm = 0;
  wss_data.fl_rpm = fl_wheel_rpm_msg.fl_rpm;
  wss_data.fr_rpm = fr_wheel_rpm_msg.fr_rpm;
  wss_data.timestamp_ = fl_wheel_rpm_msg.header.stamp;
  if (wss_data.fl_rpm < wss_data.fr_rpm - 40) {
    wss_data.fl_rpm = this->last_decent_fl_wss_reading;
  } else {
    this->last_decent_fl_wss_reading = wss_data.fl_rpm;
  }
  if (fl_wheel_rpm_msg.fl_rpm > 800 || fr_wheel_rpm_msg.fr_rpm > 800) {
    RCLCPP_WARN(this->get_logger(), "Wheel RPM is too high");
    return;
  }
  this->_velocity_estimator_->wss_callback(wss_data);
  RCLCPP_DEBUG(this->get_logger(), "Timestamp: %f", wss_data.timestamp_.seconds());
  this->publish_velocities();
}

void VehicleAdapter::steering_angle_callback(const custom_interfaces::msg::SteeringAngle msg) {
  this->_velocity_estimator_->steering_callback(msg.steering_angle);
  // this->publish_velocities();
}

void VehicleAdapter::free_acceleration_callback(
    const geometry_msgs::msg::Vector3Stamped& free_acceleration_msg) {
  common_lib::sensor_data::ImuData imu_data;
  imu_data.acceleration_x = free_acceleration_msg.vector.x;
  imu_data.acceleration_y = free_acceleration_msg.vector.y;
  imu_data.timestamp_ = free_acceleration_msg.header.stamp;
  RCLCPP_DEBUG(this->get_logger(), "Received Accelerations. Timestamp: %f",
               imu_data.timestamp_.seconds());
  this->_velocity_estimator_->imu_accel_callback(imu_data);
  this->publish_velocities();
}

void VehicleAdapter::angular_velocity_callback(
    const geometry_msgs::msg::Vector3Stamped& angular_velocity_msg) {
  common_lib::sensor_data::ImuData imu_data;
  if (this->number_of_imu_readings < 250) {
    this->average_imu_bias =
        (this->average_imu_bias * this->number_of_imu_readings + angular_velocity_msg.vector.z) /
        (this->number_of_imu_readings + 1);
    this->number_of_imu_readings++;
    return;
  }
  imu_data.rotational_velocity =
      -(angular_velocity_msg.vector.z - this->average_imu_bias);  // + 0.001838);
  imu_data.timestamp_ = angular_velocity_msg.header.stamp;
  RCLCPP_DEBUG(this->get_logger(), "Received Angular Velocities. Timestamp: %f",
               imu_data.timestamp_.seconds());
  this->_velocity_estimator_->imu_angular_callback(imu_data);
  this->publish_velocities();
}

// void VehicleAdapter::imu_callback(
//     const geometry_msgs::msg::Vector3Stamped::SharedPtr& free_acceleration_msg,
//     const geometry_msgs::msg::Vector3Stamped::SharedPtr& angular_velocity_msg) {
//   common_lib::sensor_data::ImuData imu_data;
//   if (this->number_of_imu_readings < 250) {
//     this->average_imu_bias =
//         (this->average_imu_bias * this->number_of_imu_readings + angular_velocity_msg->vector.z)
//         / (this->number_of_imu_readings + 1);
//     this->number_of_imu_readings++;
//     return;
//   }
//   imu_data.rotational_velocity =
//       -(angular_velocity_msg->vector.z - this->average_imu_bias);  // + 0.001838);
//   imu_data.acceleration_x = free_acceleration_msg->vector.x;
//   imu_data.acceleration_y = free_acceleration_msg->vector.y;
//   imu_data.timestamp_ = free_acceleration_msg->header.stamp;
//   RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "Timestamp: %f",
//                imu_data.timestamp_.seconds());
//   this->_velocity_estimator_->imu_callback(imu_data);
//   this->publish_velocities();
// }

void VehicleAdapter::resolver_callback(custom_interfaces::msg::WheelRPM msg) {
  this->_velocity_estimator_->motor_rpm_callback(msg.rr_rpm);
  // this->publish_velocities();
}