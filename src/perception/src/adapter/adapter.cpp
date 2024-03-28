#include "adapter/adapter.hpp"

#include "perception/perception_node.hpp"

Adapter::Adapter(Perception *perception) {
  this->node = perception;

  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}
