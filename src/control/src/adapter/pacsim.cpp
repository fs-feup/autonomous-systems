#include "adapter/pacsim.hpp"

PacSimAdapter::PacSimAdapter(const ControlParameters& params)
    : ControlNode(params),
      steering_pub_(create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10)),
      acceleration_pub_(
          create_publisher<pacsim::msg::StampedScalar>("/pacsim/throttle_setpoint", 10)) {
  // No topic for pacsim, just set the go_signal to true
  go_signal_ = true;

  if (this->params_.using_simulated_slam_) {
    pacsim_pose_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/pose", 1,
        std::bind(&PacSimAdapter::_pacsim_gt_pose_callback, this, std::placeholders::_1));
  }

  if (this->params_.using_simulated_velocities_) {
    this->pacsim_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 1,
        std::bind(&PacSimAdapter::_pacsim_gt_velocities_callback, this, std::placeholders::_1));
    
    this->pacsim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/pacsim/imu/cog_imu", 1,
        std::bind(&PacSimAdapter::_pacsim_imu_callback, this, std::placeholders::_1));
        
    this->pacsim_steering_angle_sub_ = this->create_subscription<pacsim::msg::StampedScalar>(
        "/pacsim/steeringFront", 1,
        std::bind(&PacSimAdapter::_pacsim_steering_angle_callback, this, std::placeholders::_1));
        
    this->pacsim_wheels_sub_ = this->create_subscription<pacsim::msg::Wheels>(
        "/pacsim/wheelspeeds", 1,
        std::bind(&PacSimAdapter::_pacsim_wheels_callback, this, std::placeholders::_1));
  }
  RCLCPP_INFO(this->get_logger(), "Pacsim adapter created");
}

void PacSimAdapter::_pacsim_gt_pose_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  custom_interfaces::msg::Pose pose;
  pose.header.stamp = msg.header.stamp;
  pose.theta = std::atan2(std::sin(msg.twist.twist.angular.z), std::cos(msg.twist.twist.angular.z));
  pose.x = msg.twist.twist.linear.x;
  pose.y = msg.twist.twist.linear.y;

  this->vehicle_pose_callback(pose);
}

void PacSimAdapter::_pacsim_gt_velocities_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  this->current_state_.header.stamp = msg.header.stamp;
  this->current_state_.velocity_x = msg.twist.twist.linear.x;
  this->current_state_.velocity_y = msg.twist.twist.linear.y;
  this->current_state_.yaw_rate = msg.twist.twist.angular.z;
  this->vehicle_state_callback(this->current_state_);
}

void PacSimAdapter::_pacsim_imu_callback(const sensor_msgs::msg::Imu& msg) {
  this->current_state_.acceleration_x = msg.linear_acceleration.x;
  this->current_state_.acceleration_y = msg.linear_acceleration.y;
  this->vehicle_state_callback(this->current_state_);
}

void PacSimAdapter::_pacsim_steering_angle_callback(const pacsim::msg::StampedScalar& msg) {
  this->current_state_.steering_angle = msg.value;
  this->vehicle_state_callback(this->current_state_);
}

void PacSimAdapter::_pacsim_wheels_callback(const pacsim::msg::Wheels& msg) {
  this->current_state_.fl_rpm = msg.fl;
  this->current_state_.fr_rpm = msg.fr;
  this->current_state_.rl_rpm = msg.rl;
  this->current_state_.rr_rpm = msg.rr;
  this->vehicle_state_callback(this->current_state_);
}

void PacSimAdapter::publish_command(common_lib::structures::ControlCommand cmd) {
  auto steering_msg = pacsim::msg::StampedScalar();
  auto acceleration_msg = pacsim::msg::StampedScalar();

  acceleration_msg.value = (cmd.throttle_rr + cmd.throttle_rl) / 2.0;
  steering_msg.value = cmd.steering_angle;

  this->steering_pub_->publish(steering_msg);
  this->acceleration_pub_->publish(acceleration_msg);
}