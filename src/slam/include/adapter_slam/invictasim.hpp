#pragma once

#include "custom_interfaces/msg/operational_status.hpp"
#include "ros_node/slam_node.hpp"

/**
 * @brief Adapter class to interface with the InvictaSim simulator for SLAM
 *
 * @details This class subscribes to the necessary topics from InvictaSim and adapts the data for use in
 * the SLAM node
 */
class InvictaSimAdapter : public SLAMNode {
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr
      _operational_status_subscription_;  ///< Subscriber for operational status

public:
  /**
   * @brief Constructor of the InvictaSim adapter node
   */
  InvictaSimAdapter(const SLAMParameters& params);

  /**
   * @brief Sends the finished signal
   */
  void finish() override;
};
