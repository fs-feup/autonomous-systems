#include <memory>

#include "node_/node_long_control.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LongitudinalControl>());
  rclcpp::shutdown();
  return 0;
}