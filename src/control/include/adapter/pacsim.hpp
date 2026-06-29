#pragma once

#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "ros_node/ros_node.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/imu.hpp"

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
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pacsim_velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pacsim_imu_sub_;
  rclcpp::Subscription<pacsim::msg::StampedScalar>::SharedPtr pacsim_steering_angle_sub_;
  rclcpp::Subscription<pacsim::msg::Wheels>::SharedPtr pacsim_wheels_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pacsim_pose_sub_;

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
   * @brief Callback for the pacsim IMU topic, used when using simulated velocity estimation and/or simulated SLAM
   * 
   * @param msg 
   */
  void _pacsim_imu_callback(const sensor_msgs::msg::Imu &msg);

  /**
   * @brief Callback for the pacsim steering angle topic, used when using simulated steering
   * 
   * @param msg 
   */
  void _pacsim_steering_angle_callback(const pacsim::msg::StampedScalar &msg);

  /**
   * @brief Callback for the pacsim wheels topic, used when using simulated velocity estimation
   * 
   * @param msg 
   */
  void _pacsim_wheels_callback(const pacsim::msg::Wheels &msg);

  void publish_command(common_lib::structures::ControlCommand cmd) override;
};