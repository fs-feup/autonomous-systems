#include "adapter/adapter.hpp"

#include "lateral_control/lateral_control_node.hpp"

Adapter::Adapter(LateralControl *lat_control) {
  this->node = lat_control;

  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}
