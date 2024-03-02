#include "adapter/adapter.hpp"

#include "planning/planning.hpp"

Adapter::Adapter(Planning *planning) {
  this->node = planning;

  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}
