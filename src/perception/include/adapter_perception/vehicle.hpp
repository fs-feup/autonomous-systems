#pragma once
#include <string>

#include "perception/perception_node.hpp"
#include "custom_interfaces/msg/operational_status.hpp"

/**
 * @brief Adapter class for Vehicle purposes.
 *
 * This class extends the Perception class and is used for testing purposes - Lidar's functionality.
 */
class VehicleAdapter : public Perception {
public:
  /**
   * @brief Constructor for TestAdapter class.
   *
   * @param params The parameters for perception.
   */
  explicit VehicleAdapter(const PerceptionParameters& params);

  /**
   * @brief Callback function for handling mission state messages.
   *
   * @param msg The mission state message.
   */
  void mission_state_callback(const custom_interfaces::msg::OperationalStatus& msg);

  /**
   * @brief Finalizes the Test Adapter.
   *
   * Overrides the virtual method in the base class.
   */
  void finish() override;
};
