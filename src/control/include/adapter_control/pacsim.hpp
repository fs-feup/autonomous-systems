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
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr acceleration_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr finished_client_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr car_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr car_pose_sub_;

  double last_stored_velocity_{0.0};

  // TODO: MISSION FINISHED IS A SERVICE NOT A TOPIC,

 public:
  explicit PacSimAdapter(const ControlParameters &params);

  /**
   * @brief Callback for the pacsim ground truth pose topic
   */
  void _pacsim_gt_pose_callback(const geometry_msgs::msg::TwistWithCovarianceStamped &msg);

  /**
   * @brief Callback for the pacsim ground truth velocity topic
   */
  void _pacsim_gt_velocities_callback(const geometry_msgs::msg::TwistWithCovarianceStamped &msg);

  /**
   * @brief send signal to finish mission
   */
  void finish();
  void publish_cmd(double acceleration = 0, double steering = 0) override;
};