#pragma once

#include <string>

#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class SENode;

/**
 * @brief Class that handles the communication between the SpeedEstimation node and the
 * other nodes in the system according to the selected mode
 *
 */
class Adapter {
public:
  std::shared_ptr<SENode> node_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription_;

  explicit Adapter(std::shared_ptr<SENode> se_node);
  virtual void finish() = 0;  ///< Function that sends the finish signal to the respective node

  /**
   * @brief Function that parses the message sent from the ros IMU topic
   * and calls the respective node's function to update the motion
   *
   * @param msg Message sent from the ros IMU topic
   */
  void imu_subscription_callback(const sensor_msgs::msg::Imu& msg) const;

  virtual ~Adapter() = default;
};
