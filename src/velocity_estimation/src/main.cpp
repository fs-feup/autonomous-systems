#include <cstdio>

#include "adapters/parameters_factory.hpp"

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
  load_adapter_parameters(params);
  auto velocity_estimator = create_ve(params);
  rclcpp::spin(velocity_estimator);
  rclcpp::shutdown();

  return 0;
}
