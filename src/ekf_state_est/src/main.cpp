#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "adapter_ekf_state_est/parameters_factory.hpp"

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  EKFStateEstParameters params;
  std::string adapter_type = load_adapter_parameters(params);
  auto ekf_state_est = create_ekf_state_est(adapter_type, params);

  rclcpp::spin(ekf_state_est);
  rclcpp::shutdown();

  return 0;
}
