#include "adapter_control/eufs.hpp"

#include "node_/node_control.hpp"

EufsAdapter::EufsAdapter(Control* control)
    : Adapter(control),
      control_pub_(
          node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10)) {
  // No topic for eufs, just set the go_signal to true
  node_->go_signal_ = true;

  if (this->node_->declare_parameter<int>("use_simulated_se", 0)) {
    RCLCPP_INFO(this->node_->get_logger(), "Eufs using simulated State Estimation\n");
    vehicle_state_sub_ = this->node_->create_subscription<eufs_msgs::msg::CarState>(
        "/odometry_integration/car_state", 10,
        std::bind(&EufsAdapter::vehicle_state_callback, this, std::placeholders::_1));
  }
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

  node_->publish_control(vehicle_state);
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
