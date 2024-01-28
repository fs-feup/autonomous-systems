#include <memory>

#include "lateral_control/lateral_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LateralControl>());
  rclcpp::shutdown();
  return 0;
}