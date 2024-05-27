#include "adapter_planning/adapter.hpp"

Adapter::Adapter(Planning* planning) {
  this->node = planning;

  RCLCPP_DEBUG(this->node->get_logger(), "Planning: Adapter created");
}
