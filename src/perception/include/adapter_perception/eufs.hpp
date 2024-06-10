#pragma once

#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"

/**
 * @brief Adapter class for interfacing with EUFS (Edinburgh University Formula Student).
 * 
 * This class extends the Perception class and provides functionality specific to
 * communication with the EUFS simulator.
 */
class EufsAdapter : public Perception {
  ///< Client for setting EUFS mission state.
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;

  ///< Client for setting EUFS ebs.
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

 public:
  /**
   * @brief Constructor for EufsAdapter class.
   * 
   * @param params The parameters for perception.
   */
  explicit EufsAdapter(const PerceptionParameters& params);

  /**
   * @brief Finalizes the EUFS Adapter.
   * 
   * Overrides the virtual method in the base class.
   */
  void finish() override;
};