#include <cstdio>

#include "adapter_slam/map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_config/general_config.hpp"

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  SLAMParameters params;
  std::string adapter_type = params.load_parameters();
  std::shared_ptr<SLAMNode> slam_node = adapter_map.at(adapter_type)();

  rclcpp::spin(slam_node);
  rclcpp::shutdown();

  return 0;
}
