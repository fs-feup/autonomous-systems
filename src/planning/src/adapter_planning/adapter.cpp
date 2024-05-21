#include "adapter_planning/adapter.hpp"

Adapter::Adapter(std::shared_ptr<Planning> planning) {
  this->node = planning;

  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}
