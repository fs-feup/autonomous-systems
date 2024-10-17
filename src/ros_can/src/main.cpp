#include <memory>

#include "../include/canlib_wrappers/can_lib_wrapper.hpp"
#include "node/node_ros_can.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Main function for the Longitudinal Control node.
 *
 * Initializes the ROS node, creates a LongitudinalControl object,
 * enters the ROS 2 event loop, and then shuts down the node when done (spin is
 * over).
 *
 * @return 0 on successful completion.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto canLibWrapper = std::make_shared<CanLibWrapper>();
  rclcpp::spin(std::make_shared<RosCan>(canLibWrapper));
  rclcpp::shutdown();
  return 0;
}