#include "loader/config.hpp"
#include "node/invictasim_node.hpp"

/**
 * @brief Main function for the invictasim node.
 *
 * Initializes the ROS node, loads configuration parameters,
 * creates the InvictaSimNode, enters the ROS 2 event loop,
 * and shuts down when done.
 *
 * @return 0 on successful completion.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  InvictaSimParameters params = load_config();
  auto invictasim_node = std::make_shared<InvictaSimNode>(params);

  rclcpp::spin(invictasim_node);
  rclcpp::shutdown();
  return 0;
}
