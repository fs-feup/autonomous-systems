#pragma once

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "rclcpp/rclcpp.hpp"

class Control;
/**
 * @brief Adapter class for coordinating communication between different modes
 * and Control module.
 */
class Adapter {
 public:
  std::shared_ptr<Control> node_;
  /**
   * @brief Constructor for the Adapter class.
   * @param mode The selected mode.
   * @param control A pointer to the Control module.
   */
  explicit Adapter(Control *control);
  virtual ~Adapter() = default;

  virtual void publish_cmd(double torque, double steering) = 0;
};
