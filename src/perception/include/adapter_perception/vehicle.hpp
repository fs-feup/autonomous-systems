#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_TEST_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_TEST_HPP_

#include <string>
#include "adapter_perception/adapter.hpp"

class Perception;

/**
 * @brief Adapter class for Vehicle purposes.
 * 
 * This class extends the Adapter class and is used for testing purposes - Lidar's functionality.
 */
class VehicleAdapter : public Adapter {
public:
  /**
   * @brief Constructor for TestAdapter class.
   * 
   * @param perception A pointer to the Perception instance.
   */
  explicit VehicleAdapter(Perception* perception);

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

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_TEST_HPP_
