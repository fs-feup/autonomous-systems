#include <memory>

#include "node_/node_long_control.hpp"
#include "rclcpp/rclcpp.hpp"
/**
 * @brief Main function for the Ros Can node.
 *
 * Initializes the ROS node, creates a Longitudinal Control object,
 * enters the ROS 2 event loop, and then shuts down the node when done (spin is
 * over).
 *
 * @return 0 on successful completion.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LongitudinalControl>());
  rclcpp::shutdown();
  return 0;
}