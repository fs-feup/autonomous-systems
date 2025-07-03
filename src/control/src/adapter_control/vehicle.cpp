#include "adapter_control/vehicle.hpp"

#include "common_lib/competition_logic/mission_logic.hpp"

VehicleAdapter::VehicleAdapter(const ControlParameters& params)
    : Control(params),
      go_sub_(create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&VehicleAdapter::go_signal_callback, this, std::placeholders::_1))),
      control_pub_(
          create_publisher<custom_interfaces::msg::ControlCommand>("/as_msgs/controls", 10)) {
  RCLCPP_INFO(this->get_logger(), "Vehicle adapter created");
  this->_lidar_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/genz/odometry", 10,
      std::bind(&VehicleAdapter::_lidar_odometry_subscription_callback, this,
                std::placeholders::_1));
}

void VehicleAdapter::publish_cmd(double acceleration, double steering) {
  auto message = custom_interfaces::msg::ControlCommand();

  // Convert values to range according to the units sent by AS
  message.throttle = acceleration;
  message.steering = steering;

  this->control_pub_->publish(message);
}

void VehicleAdapter::go_signal_callback(const custom_interfaces::msg::OperationalStatus msg) {
  // No need to do anything with the message, just set the go_signal to true
  go_signal_ = msg.go_signal;
  if (!(msg.as_mission == common_lib::competition_logic::Mission::TRACKDRIVE) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::AUTOCROSS) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::SKIDPAD) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::ACCELERATION)) {
    go_signal_ = false;
  }
}

void VehicleAdapter::_lidar_odometry_subscription_callback(const nav_msgs::msg::Odometry& msg) {
  if (!this->_first_lidar_odometry_received_) {
    this->_first_lidar_odometry_received_ = true;
    this->_last_lidar_odometry_received_ = msg;
    this->_last_lidar_odometry_received_time_ = rclcpp::Clock().now();
  }

  double variation = ::sqrt(
      ::pow(msg.pose.pose.position.x - this->_last_lidar_odometry_received_.pose.pose.position.x,
            2) +
      ::pow(msg.pose.pose.position.y - this->_last_lidar_odometry_received_.pose.pose.position.y,
            2));
  rclcpp::Time current_time = rclcpp::Clock().now();
  double delta_t =
      (current_time - this->_last_lidar_odometry_received_time_).seconds() +
      (current_time - this->_last_lidar_odometry_received_time_).nanoseconds() / 1000000000;
  this->velocity_ = variation / delta_t;
}