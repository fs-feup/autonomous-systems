#include "adapter_control/adapter.hpp"

#include "node_/node_control.hpp"

Adapter::Adapter(Control *control) : node_(control) {
  RCLCPP_INFO(this->node_->get_logger(), "Adapter created");
}
