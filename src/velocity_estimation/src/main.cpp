#include <cstdio>

#include "adapters/map.hpp"
#include "config/parameters.hpp"

/**
 * @brief Main function for the velocity estimation
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  VEParameters params;
  std::string adapter_name = params.load_config();
  auto ve_node = adapter_map.at(adapter_name)(params);
  rclcpp::spin(ve_node);
  rclcpp::shutdown();

  return 0;
}
