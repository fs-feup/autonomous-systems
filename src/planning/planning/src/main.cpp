#include "planning/planning.hpp"

/**
 * @brief Main function for the planning node.
 *
 * Initializes the ROS node, creates a Planning object,
 * enters the ROS 2 event loop, and then shuts down the node when done (spin is over).
 *
 * @return 0 on successful completion.
 */

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto planning = std::make_shared<Planning>();
  rclcpp::spin(planning);
  rclcpp::shutdown();
  return 0;
}