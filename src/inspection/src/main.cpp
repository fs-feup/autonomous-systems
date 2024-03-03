#include <cstdio>
#include "inspection/inspection.hpp"


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InspectionMission>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
