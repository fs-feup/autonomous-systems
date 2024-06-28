#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "ros_node/se_node.hpp"

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  auto speed_est = std::make_shared<SENode>();
  rclcpp::spin(speed_est);
  rclcpp::shutdown();

  return 0;
}
