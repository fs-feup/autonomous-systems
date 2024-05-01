#include "src/long_control/include/adapter/adapter.hpp"

#include "src/long_control/include/node_/node_long_control.hpp"

Adapter::Adapter(LongitudinalControl *long_control) {
  this->node = long_control;

  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}
