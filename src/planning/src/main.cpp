#include "adapter_planning/eufs.hpp"
#include "adapter_planning/fsds.hpp"
#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/parameters_factory.hpp"
#include "adapter_planning/vehicle.hpp"
#include "planning/planning.hpp"

/**
 * @brief Main function for the planning node.
 *
 * Initializes the ROS node, creates a Planning object,
 * enters the ROS 2 event loop, and then shuts down the node when done (spin is
 * over).
 *
 * @return 0 on successful completion.
 */

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  PlanningParameters params;

  std::string adapter_type = load_adapter_parameters(params);
  std::shared_ptr<Planning> planning = create_planning(adapter_type, params);
  if (!planning) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to create planning object");
    return 1;
  }

  rclcpp::spin(planning);
  rclcpp::shutdown();
  return 0;
}