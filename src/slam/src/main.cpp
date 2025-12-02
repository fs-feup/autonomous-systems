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
  std::string adapter_type = params.load_config();
  std::shared_ptr<SLAMNode> slam_node = adapter_map.at(adapter_type)(params);
  slam_node->init();

  // If graph slam, use a multi-threaded executor
  if (params.slam_solver_name_ != "ekf_slam") {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(slam_node);
    executor.spin();
  } else {
    rclcpp::spin(slam_node);
  }
  rclcpp::shutdown();

  return 0;
}
