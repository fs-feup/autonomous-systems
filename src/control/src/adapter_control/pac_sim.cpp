#include "adapter_control/pac_sim.hpp"

#include "node_/node_control.hpp"

PacSimAdapter::PacSimAdapter(bool using_simulated_se, bool mocker_node, double lookahead_gain,
                             double lookahead_margin)
    : Control(using_simulated_se, mocker_node, lookahead_gain, lookahead_margin),
      steering_pub_(create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10)),
      acceleration_pub_(create_publisher<pacsim::msg::Wheels>("/pacsim/torques_max", 10)) {
  // No topic for pacsim, just set the go_signal to true
  go_signal_ = true;

  if (using_simulated_se_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Maybe change time to a lower value if needed: std::chrono::milliseconds(10)
    RCLCPP_INFO(this->get_logger(), "Creating wall timer");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                                     std::bind(&PacSimAdapter::timer_callback, this));

    this->finished_client_ = this->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");

    car_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 10, [this](const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
          last_stored_velocity_ = std::sqrt(std::pow(msg.twist.twist.linear.x, 2) +
                                            std::pow(msg.twist.twist.linear.y, 2) +
                                            std::pow(msg.twist.twist.linear.z, 2));
          // RCLCPP_INFO(this->get_logger(), "velocity_callback: storing current
          // velocity");
        });
  }

  RCLCPP_INFO(this->get_logger(), "Pacsim adapter created");
}

void PacSimAdapter::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "timer_callback: trying to call canTransform");
  if (tf_buffer_->canTransform("map", "car", tf2::TimePointZero)) {
    RCLCPP_INFO(this->get_logger(), "timer_callback: canTransform success");
    custom_interfaces::msg::VehicleState pose;
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
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.linear_velocity = this->last_stored_velocity_;
    pose.angular_velocity = 0.0;  // not needed -> default value

    RCLCPP_INFO(get_logger(), "Pose info. Position:%f, %f;  Linear velocity %f, Theta %f",
                pose.position.x, pose.position.y, pose.linear_velocity, pose.theta);
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
  auto acceleration_msg = pacsim::msg::Wheels();

  // TODO: Convert values if necessary then fill the messages
  //  CODE HERE
  acceleration_msg.fl = acceleration_msg.fr = acceleration_msg.rl = acceleration_msg.rr =
      acceleration;
  steering_msg.value = steering;

  this->steering_pub_->publish(steering_msg);
  this->acceleration_pub_->publish(acceleration_msg);
}
