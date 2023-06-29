#include "can.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    printf("Too few arguments!\r\n");
    printf("Usage: ros2 run can can <mode>\r\n");
    return 1;
  }

  rclcpp::spin(std::make_shared<Can>());
  rclcpp::shutdown();

  return 0;
}