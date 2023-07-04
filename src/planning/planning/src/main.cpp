#include "planning/planning.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto planning = std::make_shared<Planning>();
  try {
    rclcpp::spin(planning);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(planning->get_logger(), "%s", e.what());
    throw std::runtime_error("Planning runtime error!");
  }
  rclcpp::shutdown();
  std::cout << "Finished properly!\n";
  return 0;
}