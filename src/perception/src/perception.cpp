#include <cstdio>

#include "perception/perception_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Perception>();

  RCLCPP_INFO(node->get_logger(), "Perception is alive!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}