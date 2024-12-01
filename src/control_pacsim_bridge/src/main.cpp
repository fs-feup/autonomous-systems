#include "node/bridge.hpp"


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}