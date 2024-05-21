#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "planning/planning.hpp"
#include "rclcpp/rclcpp.hpp"

class Planning;

/**
 * @brief Adapter class for coordinating communication between different modes
 * and Planning module.
 */
class Adapter {
public:
  explicit Adapter(std::shared_ptr<Planning> planning);
  std::shared_ptr<Planning> node;

  virtual void finish() = 0;
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
