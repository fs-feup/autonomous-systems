#include "adapter/map.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  ControlParameters params;
  std::string adapter = params.load_config();

  std::shared_ptr<ControlNode> control_node = adapter_map.at(adapter)(params);

  rclcpp::spin(control_node);
  rclcpp::shutdown();
  return 0;
}