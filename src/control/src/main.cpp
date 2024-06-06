#include <memory>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
#include "adapter_control/pac_sim.hpp"
#include "adapter_control/vehicle.hpp"
#include "node_/node_control.hpp"
#include "rclcpp/rclcpp.hpp"

void load_adapter_parameters(bool& using_simulated_se, bool& mocker_node, double& lookahead_gain,
                             double& lookahead_margin, std::string& adapter_type) {
  auto adapter_node = std::make_shared<rclcpp::Node>("control_adapter");
  using_simulated_se = adapter_node->declare_parameter("use_simulated_se", false);
  mocker_node = adapter_node->declare_parameter("mocker_node", true);
  lookahead_gain = adapter_node->declare_parameter("lookahead_gain", 0.5);
  lookahead_margin = adapter_node->declare_parameter("lookahead_margin", 0.1);

  adapter_type = adapter_node->declare_parameter("adapter", "pacsim");
}

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
  std::shared_ptr<Control> control;
  std::string adapter_type;

  bool using_simulated_se;
  bool mocker_node;
  double lookahead_gain;
  double lookahead_margin;

  load_adapter_parameters(using_simulated_se, mocker_node, lookahead_gain, lookahead_margin,
                          adapter_type);

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
  }
  rclcpp::spin(control);
  rclcpp::shutdown();
  return 0;
}