#include "adapter/pacsim.hpp"

PacSimAdapter::PacSimAdapter(const ControlParameters& params)
    : ControlNode(params),
      steering_pub_(create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10)),
      throttle_pub_(create_publisher<pacsim::msg::Wheels>("/pacsim/throttle_setpoint", 10)) {
  // No topic for pacsim, just set the go_signal to true
  go_signal_ = true;

  if (this->params_.using_simulated_slam_) {
    car_pose_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/pose", 1,
        std::bind(&PacSimAdapter::_pacsim_gt_pose_callback, this, std::placeholders::_1));
  }

  if (this->params_.using_simulated_velocities_) {
    car_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 1,
        std::bind(&PacSimAdapter::_pacsim_gt_velocities_callback, this, std::placeholders::_1));
    car_state_vector_sub_ = this->create_subscription<custom_interfaces::msg::VehicleStateVector>(
        "/pacsim/state_vector", 1,
        std::bind(&PacSimAdapter::_pacsim_gt_state_vector_callback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(this->get_logger(), "Pacsim adapter created");
}

void PacSimAdapter::_pacsim_gt_pose_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  custom_interfaces::msg::Pose pose;
  pose.header.stamp = msg.header.stamp;
  pose.theta = msg.twist.twist.angular.z;
  pose.x = msg.twist.twist.linear.x;
  pose.y = msg.twist.twist.linear.y;

  this->vehicle_pose_callback(pose);
}

void PacSimAdapter::_pacsim_gt_velocities_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  custom_interfaces::msg::Velocities vel_msg;
  vel_msg.header.stamp = msg.header.stamp;
  vel_msg.velocity_x = msg.twist.twist.linear.x;
  vel_msg.velocity_y = msg.twist.twist.linear.y;
  vel_msg.angular_velocity = msg.twist.twist.angular.z;
  this->vehicle_state_callback(vel_msg);
}

void PacSimAdapter::_pacsim_gt_state_vector_callback(
    const custom_interfaces::msg::VehicleStateVector& msg) {
  // Currently Velocities is used, not VehicleStateVector, but this is here for future use
}

void PacSimAdapter::publish_command(common_lib::structures::ControlCommand cmd) {
  auto steering_msg = pacsim::msg::StampedScalar();
  auto throttle_msg = pacsim::msg::Wheels();

  throttle_msg.fl = cmd.throttle_fl;
  throttle_msg.fr = cmd.throttle_fr;
  throttle_msg.rl = cmd.throttle_rl;
  throttle_msg.rr = cmd.throttle_rr;
  steering_msg.value = cmd.steering_angle;

  this->steering_pub_->publish(steering_msg);
  this->throttle_pub_->publish(throttle_msg);
}