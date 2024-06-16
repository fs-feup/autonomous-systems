#include <memory>

#include "adapter_control/parameters_factory.hpp"
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

  ControlParameters params;
  std::string adapter_type = load_adapter_parameters(params);
  auto control = create_control(adapter_type, params);

  if (!control) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "Failed to create control object");
    return 1;
  }

  rclcpp::spin(control);
  rclcpp::shutdown();
  return 0;
}