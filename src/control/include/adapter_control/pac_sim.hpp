#pragma once

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "node_/node_control.hpp"


class PacSimAdapter : public Control {
 private:
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_pub_;
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr acceleration_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr finished_client_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr car_velocity_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double last_stored_velocity_{0.0};

  // TODO: MISSION FINISHED IS A SERVICE NOT A TOPIC,

 public:
  explicit PacSimAdapter(bool using_simulated_se, bool mocker_node, double lookahead_gain, double lookahead_margin);
  void timer_callback();
  void finish();
  void publish_cmd(double acceleration = 0, double steering = 0) override;
};