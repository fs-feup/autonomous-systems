#include "adapter_control/adapter.hpp"

#include "node_/node_control.hpp"

Adapter::Adapter(Control *control) : node(control) {
  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}
