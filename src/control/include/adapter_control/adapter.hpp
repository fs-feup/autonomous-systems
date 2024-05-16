#ifndef SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "rclcpp/rclcpp.hpp"

class Control;
/**
 * @brief Adapter class for coordinating communication between different modes
 * and Control module.
 */
class Adapter {
 protected:
  Control *node;

 public:
  /**
   * @brief Constructor for the Adapter class.
   * @param mode The selected mode.
   * @param control A pointer to the Control module.
   */
  explicit Adapter(Control *control);
  virtual ~Adapter() = default;

  virtual void finish() = 0;
  virtual void publish_cmd(float acceleration, float steering) = 0;
};

#endif  // SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
