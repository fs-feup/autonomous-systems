#include <memory>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
#include "adapter_control/map.hpp"
#include "adapter_control/pac_sim.hpp"
#include "adapter_control/vehicle.hpp"
#include "node_/node_control.hpp"
#include "rclcpp/rclcpp.hpp"
/**
 * @brief Main function for the Ros Can node.
 *
 * Initializes the ROS node, creates a Control object,
 * enters the ROS 2 event loop, and then shuts down the node when done (spin is
 * over).
 *
 * @return 0 on successful completion.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto adapter_node = std::make_shared<rclcpp::Node>("control_adapter");
  rclcpp::spin(adapter_map.at(adapter_node->declare_parameter("adapter", "pacsim"))());
  rclcpp::shutdown();
  return 0;
}