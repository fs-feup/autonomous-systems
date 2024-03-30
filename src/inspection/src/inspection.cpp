#include <cstdio>

#include "include/inspection_ros.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InspectionMission>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), e.what());
  }
  rclcpp::shutdown();
  return 0;
}
