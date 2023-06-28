#include "orchestrator.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    printf("Too few arguments!\r\n");
    printf("Usage: ros2 run orchestrator orchestrator <mode>\r\n");
    return 1;
  }

  rclcpp::spin(std::make_shared<Orchestrator>(argv[1]));
  rclcpp::shutdown();

  return 0;
}