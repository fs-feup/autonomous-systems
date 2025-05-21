#include "adapter_control/pacsim.hpp"

#include "custom_interfaces/msg/pose.hpp"
#include "node_/node_control.hpp"

PacSimAdapter::PacSimAdapter(const ControlParameters& params)
    : Control(params),
      steering_pub_(create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10)),
      acceleration_pub_(
          create_publisher<pacsim::msg::StampedScalar>("/pacsim/throttle_setpoint", 10)) {
  // No topic for pacsim, just set the go_signal to true
  go_signal_ = true;

  this->finished_client_ = this->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");
  if (using_simulated_slam_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Maybe change time to a lower value if needed: std::chrono::milliseconds(10)
    RCLCPP_INFO(this->get_logger(), "Creating wall timer");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&PacSimAdapter::timer_callback, this));
  }

  if (using_simulated_velocities_) {
    car_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 10, [this](const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
          velocity_ = std::sqrt(std::pow(msg.twist.twist.linear.x, 2) +
                                std::pow(msg.twist.twist.linear.y, 2) +
                                std::pow(msg.twist.twist.linear.z, 2));
        });
  }

  RCLCPP_INFO(this->get_logger(), "Pacsim adapter created");
}

void PacSimAdapter::timer_callback() {
  if (tf_buffer_->canTransform("map", "car", tf2::TimePointZero)) {
    custom_interfaces::msg::Pose pose;
    geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform("map", "car", tf2::TimePointZero);
    pose.header = t.header;
    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                      t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);
    pose.theta = yaw;
    pose.x = t.transform.translation.x;
    pose.y = t.transform.translation.y;

    RCLCPP_DEBUG(get_logger(), "Pose info. Position:%f, %f, Theta %f", pose.x, pose.y, pose.theta);
    this->publish_control(pose);
  }
}

void PacSimAdapter::finish() {
  this->finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
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