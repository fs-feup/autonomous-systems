#pragma once

#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "ros_node/ros_node.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @brief Adapter for the PacSim simulator. Works on a publish-subscribe model.
 *
 */
class PacSimAdapter : public ControlNode {
private:
  // Publishers of commands to PacSim
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_pub_;
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr throttle_pub_;

  // If using simulated (ground truth) State Estimation (and SLAM) from PacSim
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr car_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr car_pose_sub_;
  rclcpp::Subscription<custom_interfaces::msg::VehicleStateVector>::SharedPtr car_state_vector_sub_;

public:
  explicit PacSimAdapter(const ControlParameters &params);

  /**
   * @brief Callback for the pacsim ground truth pose topic, used when using simulated SLAM
   */
  void _pacsim_gt_pose_callback(const geometry_msgs::msg::TwistWithCovarianceStamped &msg);

  /**
   * @brief Callback for the pacsim ground truth velocity topic, used when using simulated velocity
   * estimation
   */
  void _pacsim_gt_velocities_callback(const geometry_msgs::msg::TwistWithCovarianceStamped &msg);

  /**
   * @brief Callback for the pacsim ground truth state vector topic, used when using simulated full
   * state estimation
   */
  void _pacsim_gt_state_vector_callback(const custom_interfaces::msg::VehicleStateVector &msg);

  void publish_command(common_lib::structures::ControlCommand cmd) override;
};