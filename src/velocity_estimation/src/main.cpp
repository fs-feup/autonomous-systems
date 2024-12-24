#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "node/node.hpp"

/**
 * @brief Main function for the velocity estimation
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  auto velocity_estimator = std::make_shared<VENode>();
  rclcpp::spin(velocity_estimator);
  rclcpp::shutdown();

  return 0;
}
