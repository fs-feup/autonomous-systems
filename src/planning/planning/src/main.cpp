#include "planning/planning.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto planning = std::make_shared<Planning>();
  rclcpp::spin(planning);
  rclcpp::shutdown();
  return 0;
}