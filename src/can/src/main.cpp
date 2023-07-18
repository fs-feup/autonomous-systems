#include "can/can.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Can>());
  rclcpp::shutdown();

  return 0;
}