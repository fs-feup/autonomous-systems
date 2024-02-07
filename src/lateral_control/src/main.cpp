#include <memory>

#include "lateral_control/lateral_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Main Lateral Control node function
 *
 * Initiate the node, run the node and then shutdown it.
 *
 * @param argc
 * @param argv
 * @return int (0)
 */

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LateralControl>());
  rclcpp::shutdown();
  return 0;
}