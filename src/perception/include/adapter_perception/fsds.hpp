#pragma once

#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/finished_signal.hpp"


/**
 * @brief Adapter class for interfacing with FSDS (Formula Student Driverless Simulator).
 * 
 * This class extends the Perception class and provides functionality specific to
 * communication with the FSDS simulator.
 */
class FsdsAdapter : public Perception {
  ///< Subscription for receiving FSDS state.
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr fsds_state_subscription_;

  ///< Publisher for sending FSDS ebs.
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

 public:
  /**
   * @brief Constructor for FsdsAdapter class.
   * 
   * @param params The parameters for perception.
   */
  explicit FsdsAdapter(const PerceptionParameters& params);

  /**
   * @brief Callback function for handling mission state messages from FSDS.
   * 
   * @param msg The message containing FSDS mission state.
   */
  void mission_state_callback(const fs_msgs::msg::GoSignal msg);

  /**
   * @brief Finalizes the FSDS Adapter.
   * 
   * Overrides the virtual method in the base class.
   */
  void finish() override;
};
