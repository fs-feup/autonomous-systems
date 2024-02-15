#ifndef SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

class LateralControl;
/**
 * @brief Adapter class for coordinating communication between different modes
 * and LateralControl module.
 */
class Adapter {
  LateralControl *node;
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

  void fsds_publish_cmd(float steering);

  void publish_cmd(float steering);

  void fsds_finish();

 public:
  /**
   * @brief Constructor for the Adapter class.
   * @param mode The selected mode.
   * @param lat_control A pointer to the LateralControl module.
   */
  Adapter(std::string mode, LateralControl *lat_control);
};

#endif  // SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
