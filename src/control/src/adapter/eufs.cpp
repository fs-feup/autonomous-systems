#include "adapter/eufs.hpp"

EufsAdapter::EufsAdapter(const ControlParameters& params)
    : ControlNode(params),
      control_pub_(create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10)) {
  // No topic for eufs, just set the go_signal to true
  go_signal_ = true;

  if (this->params_.using_simulated_slam_) {
    RCLCPP_INFO(this->get_logger(), "Eufs using simulated State Estimation\n");
    vehicle_pose_sub_ = this->create_subscription<eufs_msgs::msg::CarState>(
        "/odometry_integration/car_state", 10,
        std::bind(&EufsAdapter::vehicle_state_callback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(this->get_logger(), "EUFS adapter created");
}

void EufsAdapter::vehicle_state_callback(const eufs_msgs::msg::CarState& msg) {
  // Update the vehicle state
  custom_interfaces::msg::Pose vehicle_state;
  vehicle_state.x = msg.pose.pose.position.x;
  vehicle_state.y = msg.pose.pose.position.y;
  vehicle_state.theta = atan2(2.0f * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                                      msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                              msg.pose.pose.orientation.w * msg.pose.pose.orientation.w +
                                  msg.pose.pose.orientation.x * msg.pose.pose.orientation.x -
                                  msg.pose.pose.orientation.y * msg.pose.pose.orientation.y -
                                  msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);

  this->vehicle_pose_callback(vehicle_state);
}

void EufsAdapter::publish_command(common_lib::structures::ControlCommand cmd) {
  auto control_msg = ackermann_msgs::msg::AckermannDriveStamped();

  // Maybe normalize values if needed??
  control_msg.drive.acceleration = (cmd.throttle_rr + cmd.throttle_rl) / 2.0;
  control_msg.drive.steering_angle = cmd.steering_angle;
  // control_msg.drive.jerk and control_msg.drive.steering_angle_velocity
  // should maybe be filled as well based on PID

  this->control_pub_->publish(control_msg);
}