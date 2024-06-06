#include "adapter_control/eufs.hpp"
#include "node_/node_control.hpp"

EufsAdapter::EufsAdapter()
    : Control(),
      control_pub_(
          create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10)) {
  // No topic for eufs, just set the go_signal to true
  go_signal_ = true;

  if (using_simulated_se_) {
    RCLCPP_INFO(this->get_logger(), "Eufs using simulated State Estimation\n");
    vehicle_state_sub_ = this->create_subscription<eufs_msgs::msg::CarState>(
        "/odometry_integration/car_state", 10,
        std::bind(&EufsAdapter::vehicle_state_callback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(this->get_logger(), "EUFS adapter created");
}

void EufsAdapter::vehicle_state_callback(const eufs_msgs::msg::CarState& msg) {
  // Update the vehicle state
  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = msg.pose.pose.position.x;
  vehicle_state.position.y = msg.pose.pose.position.y;
  vehicle_state.theta = atan2(2.0f * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                                      msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                              msg.pose.pose.orientation.w * msg.pose.pose.orientation.w +
                                  msg.pose.pose.orientation.x * msg.pose.pose.orientation.x -
                                  msg.pose.pose.orientation.y * msg.pose.pose.orientation.y -
                                  msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);


  vehicle_state.linear_velocity = std::sqrt(std::pow(msg.twist.twist.linear.x, 2) + std::pow(msg.twist.twist.linear.y, 2));

  publish_control(vehicle_state);
}

void EufsAdapter::publish_cmd(double torque, double steering) {
  auto control_msg = ackermann_msgs::msg::AckermannDriveStamped();

  // Maybe normalize values if needed??
  control_msg.drive.acceleration = torque;
  control_msg.drive.steering_angle = steering;
  // control_msg.drive.jerk and control_msg.drive.steering_angle_velocity
  // should maybe be filled as well based on PID

  this->control_pub_->publish(control_msg);
}
