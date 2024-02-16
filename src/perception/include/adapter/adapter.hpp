#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "adapter/adapter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

class Perception;
/**
 * @brief Adapter class for coordinating communication between different modes
 * and LongitudinalControl module.
 */
class Adapter {
  Perception *node;
  std::string mode;
  Adapter* adapter;

  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscription;  ///< PointCloud2 subscription.

  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  /**
   * @brief Initialize communication interfaces for FSDS mode.
   */
  void fsds_init();

  void livox_init();

  /**
   * @brief Callback for FSDS mode mission state updates.
   * @param msg The received GoSignal message.
   */
  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal);

  void fsds_finish();

 public:
  /**
   * @brief Constructor for the Adapter class.
   * @param mode The selected mode.
   * @param long_control A pointer to the LongitudinalControl module.
   */
  Adapter(std::string mode, Perception *perception);
};

#endif  // SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_
