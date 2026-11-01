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

  // If asynchronous optimization or synch parallel with graph slam, use a multi-threaded executor
  if (params.slam_optimization_mode_ == "async" ||
      (params.slam_solver_name_ == "graph_slam" &&
       params.slam_optimization_mode_ == "sync-parallel")) {
    // Bounded on purpose: the default is one thread per core, so behaviour changed between a
    // 16 core desktop and the 40 core runner. Only a handful of callback groups exist, so more
    // threads buy nothing and make the timing machine dependent.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(slam_node);
    executor.spin();
  } else {
    rclcpp::spin(slam_node);
  }
  rclcpp::shutdown();

  return 0;
}
