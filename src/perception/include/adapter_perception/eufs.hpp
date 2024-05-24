#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_EUFS_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_EUFS_HPP_

#include "adapter_perception/adapter.hpp"
#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"

class Perception;

/**
 * @brief Adapter class for interfacing with EUFS (Edinburgh University Formula Student).
 * 
 * This class extends the Adapter class and provides functionality specific to
 * communication with the EUFS simulator.
 */
class EufsAdapter : public Adapter {
  ///< Client for setting EUFS mission state.
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;

  ///< Client for setting EUFS ebs.
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

 public:
  /**
   * @brief Constructor for EufsAdapter class.
   * 
   * @param perception A pointer to the Perception instance.
   */
  explicit EufsAdapter(Perception* perception);

  /**
   * @brief Finalizes the EUFS Adapter.
   * 
   * Overrides the virtual method in the base class.
   */
  void finish() override;
};

#endif  // SRC_PERCEPTION_INCLUDE_ADAPTER_EUFS_HPP_
