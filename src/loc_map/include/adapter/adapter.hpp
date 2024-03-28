#ifndef SRC_LOC_MAP_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_LOC_MAP_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "loc_map/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class LMNode;

/**
 * @brief Class that handles the communication between the loc_map node and the
 * other nodes in the system according to the selected mode
 *
 */
class Adapter {
 public:
  LMNode *node;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription;

  explicit Adapter(LMNode *loc_map);
  virtual void init() = 0;
  virtual void finish() = 0;

  /**
   * @brief Function that parses the message sent from the ros IMU topic
   * and calls the respective node's function to update the motion
   *
   * @param msg Message sent from the ros IMU topic
   */
  void imu_subscription_callback(const sensor_msgs::msg::Imu msg);
};

#endif  // SRC_LOC_MAP_INCLUDE_ADAPTER_ADAPTER_HPP_
