#include "node/mocker_node.hpp"

/**
 * @brief Main function for the planning node.
 *
 * Initializes the ROS node, creates a Planning object,
 * enters the ROS 2 event loop, and then shuts down the node when done (spin is
 * over).
 *
 * @return 0 on successful completion.
 */

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Usage with inputs: ros2 run mocker_node mocker_node "
              "--ros-args -p track_name:=<track_name> -p sim:=<sim>");

  std::string track_name;
  std::string sim;
  {
    auto mocker_node_interface = std::make_shared<rclcpp::Node>("mocker_node_interface");
    track_name = mocker_node_interface->declare_parameter("track_name", "small_track");
    sim = mocker_node_interface->declare_parameter("sim", "pacsim");
  }

  auto node = std::make_shared<MockerNode>(track_name, sim);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}