#include <cstdio>

#include "loc_map/minimal_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();

  return 0;
}
