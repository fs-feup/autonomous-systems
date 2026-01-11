#include "config/config_loader.hpp"
#include "node/fsfsim_node.hpp"

/**
 * @brief Main function for the fsfsim node.
 *
 * Initializes the ROS node, loads configuration parameters,
 * creates the FsfsimNode, enters the ROS 2 event loop,
 * and shuts down when done.
 *
 * @return 0 on successful completion.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  FsfsimParameters params = fsfsim::load_config();
  auto fsfsim_node = std::make_shared<FsfsimNode>(params);

  rclcpp::spin(fsfsim_node);
  rclcpp::shutdown();
  return 0;
}
