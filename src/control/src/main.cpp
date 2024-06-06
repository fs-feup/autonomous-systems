#include <memory>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
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
  auto using_simulated_se = adapter_node->declare_parameter("use_simulated_se", false);
  auto mocker_node = adapter_node->declare_parameter("mocker_node", true);
  auto lookahead_gain = adapter_node->declare_parameter("lookahead_gain", 0.5);
  auto lookahead_margin = adapter_node->declare_parameter("lookahead_margin", 0.1);

  auto adapter_type = adapter_node->declare_parameter("adapter", "pacsim");
  std::shared_ptr<Control> control;
  if (adapter_type == "fsds") {
    control = std::make_shared<FsdsAdapter>(using_simulated_se, mocker_node, lookahead_gain,
                                            lookahead_margin);
  } else if (adapter_type == "pacsim") {
    control = std::make_shared<PacSimAdapter>(using_simulated_se, mocker_node, lookahead_gain,
                                              lookahead_margin);
  } else if (adapter_type == "eufs") {
    control = std::make_shared<EufsAdapter>(using_simulated_se, mocker_node, lookahead_gain,
                                            lookahead_margin);
  } else if (adapter_type == "vehicle") {
    control = std::make_shared<VehicleAdapter>(using_simulated_se, mocker_node, lookahead_gain,
                                               lookahead_margin);
  } else {
    RCLCPP_ERROR(adapter_node->get_logger(), "Invalid adapter type: %s", adapter_type.c_str());
    return 1;
  }
  
  rclcpp::spin(control);
  rclcpp::shutdown();
  return 0;
}