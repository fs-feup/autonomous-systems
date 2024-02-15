#ifndef SRC_LONG_CONTROL_LONG_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_LONG_CONTROL_LONG_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

class LongitudinalControl;
/**
 * @brief Adapter class for coordinating communication between different modes
 * and LongitudinalControl module.
 */
class Adapter {
  LongitudinalControl *node;
  std::string mode;

  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr fsds_cmd_publisher_;

  /**
   * @brief Initialize communication interfaces for FSDS mode.
   */
  void fsds_init();

  /**
   * @brief Callback for FSDS mode mission state updates.
   * @param msg The received GoSignal message.
   */
  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal);

  void fsds_publish_cmd(float throttle, float brake);

  void publish_cmd(float throttle, float brake);

  void fsds_finish();

 public:
  /**
   * @brief Constructor for the Adapter class.
   * @param mode The selected mode.
   * @param long_control A pointer to the LongitudinalControl module.
   */
  Adapter(std::string mode, LongitudinalControl *long_control);
};

#endif  // SRC_LONG_CONTROL_LONG_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
