#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "rclcpp/rclcpp.hpp"

class Planning;
/**
 * @brief Adapter class for coordinating communication between different modes and Planning module.
 */
class Adapter {
  Planning* node;

  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr eufs_state_subscription_;
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::Vcu>::SharedPtr ads_dv_state_subscription_;

  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  /**
   * @brief Initialize communication interfaces for EUFS mode.
   */
  void eufs_init();

  /**
   * @brief Initialize communication interfaces for FSDS mode.
   */
  void fsds_init();

  /**
   * @brief Initialize communication interfaces for ADS DV mode.
   */
  void ads_dv_init();

  /**
   * @brief Callback for EUFS mode mission state updates.
   * @param msg The received CanState message.
   */
  void eufs_mission_state_callback(const eufs_msgs::msg::CanState msg);

  /**
   * @brief Callback for FSDS mode mission state updates.
   * @param msg The received GoSignal message.
   */
  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal);

  /**
   * @brief Callback for ADS DV mode mission state updates.
   * @param msg The received Vcu message.
   */
  void ads_dv_mission_state_callback(const custom_interfaces::msg::Vcu msg);

  /**
   * @brief Set mission state for EUFS mode.
   * @param mission The mission to set.
   * @param state The state to set.
   */
  void eufs_set_mission_state(int mission, int state);

 public:
  /**
   * @brief Constructor for the Adapter class.
   * @param mode The selected mode.
   * @param planning A pointer to the Planning module.
   */
  Adapter(std::string mode, Planning* planning);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
