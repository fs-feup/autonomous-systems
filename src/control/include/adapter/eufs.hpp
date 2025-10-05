#pragma once

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "ros_node/ros_node.hpp"

/**
 * @brief Adapter class to interface with the EUFS simulator.
 *
 * @details Works on a publish-subscribe model. Publishes commands to the simulator
 * in the form of AckermannDriveStamped messages and subscribes to CarState messages
 * if using simulated SLAM.
 */
class EufsAdapter : public ControlNode {
private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub_;
  rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr vehicle_pose_sub_;

public:
  explicit EufsAdapter(const ControlParameters& params);
  /**
   * @brief Callback function for velocities and poses, which are coupled in EUFS.
   */
  void vehicle_state_callback(const eufs_msgs::msg::CarState& msg);
  void publish_command(common_lib::structures::ControlCommand cmd) override;
};