#include "adapter/pacsim.hpp"

#include "custom_interfaces/msg/pose.hpp"
#include "ros_node/ros_node.hpp"

PacSimAdapter::PacSimAdapter(const ControlParameters& params)
    : ControlNode(params),
      steering_pub_(create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10)),
      acceleration_pub_(
          create_publisher<pacsim::msg::StampedScalar>("/pacsim/throttle_setpoint", 10)) {
  // No topic for pacsim, just set the go_signal to true
  go_signal_ = true;

  this->finished_client_ = this->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");
  if (this->params_.using_simulated_slam_) {
    car_pose_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/pose", 1,
        std::bind(&PacSimAdapter::_pacsim_gt_pose_callback, this, std::placeholders::_1));
  }

  if (this->params_.using_simulated_velocities_) {
    car_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 1,
        std::bind(&PacSimAdapter::_pacsim_gt_velocities_callback, this, std::placeholders::_1));
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

  RCLCPP_DEBUG(get_logger(), "Pose info. Position:%f, %f, Theta %f", pose.x, pose.y, pose.theta);
  this->publish_control(pose);
}

void PacSimAdapter::_pacsim_gt_velocities_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  velocity_ =
      std::sqrt(std::pow(msg.twist.twist.linear.x, 2) + std::pow(msg.twist.twist.linear.y, 2));
  RCLCPP_DEBUG(get_logger(), "Velocity set to: %lf", velocity_);
}

void PacSimAdapter::finish() {
  this->finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        RCLCPP_INFO(this->get_logger(), "Finished signal sent");
      });
}

void PacSimAdapter::publish_cmd(double acceleration, double steering) {
  auto steering_msg = pacsim::msg::StampedScalar();
  auto acceleration_msg = pacsim::msg::StampedScalar();

  acceleration_msg.value = acceleration;
  steering_msg.value = steering;

  this->steering_pub_->publish(steering_msg);
  this->acceleration_pub_->publish(acceleration_msg);
}