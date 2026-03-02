#include <cstdio>

#include "adapters/map.hpp"
#include "utils/parameters.hpp"

/**
 * @brief Main function for the state estimation
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  std::shared_ptr<SEParameters> params = std::make_shared<SEParameters>();
  std::string adapter_name = params->load_config();
  auto se_node = adapter_map.at(adapter_name)(params);
  rclcpp::spin(se_node);
  rclcpp::shutdown();

  return 0;
}
