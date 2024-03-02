#ifndef SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "rclcpp/rclcpp.hpp"

class LongitudinalControl;
/**
 * @brief Adapter class for coordinating communication between different modes
 * and LongitudinalControl module.
 */
class Adapter {
 protected:
  LongitudinalControl *node;

 public:
  /**
   * @brief Constructor for the Adapter class.
   * @param mode The selected mode.
   * @param long_control A pointer to the LongitudinalControl module.
   */
  explicit Adapter(LongitudinalControl *long_control);

  virtual void init() = 0;
  virtual void finish() = 0;
  virtual void publish_cmd(float acceleration, float braking, float steering) = 0;
};

#endif  // SRC_LAT_CONTROL_LAT_CONTROL_INCLUDE_ADAPTER_ADAPTER_HPP_
