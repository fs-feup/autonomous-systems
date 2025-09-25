#include <cstdio>
#include <memory>

#include "perception/perception_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  PerceptionParameters params = Perception::load_config();
  auto perception = std::make_shared<Perception>(params);
  rclcpp::spin(perception);  // Spin the perception node
  rclcpp::shutdown();
  return 0;
}
