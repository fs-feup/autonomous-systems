#include <iostream>
#include <memory>
#include <optional>
#include <thread>

#include "config/config.hpp"
#include "io/input/map.hpp"
#include "io/output/map.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "simulator/invictasim.hpp"
#include "tuning/tuning_evaluator.hpp"

/**
 * @brief Initializes the simulator.
 *
 * Initializes the input and output adapters based on the configuration, creates the simulator
 * instance, and starts the simulation loop. Also handles ROS spinning when using ROS-based
 * adapters. Simulation loop, input reading, and output publishing run in separate threads.
 *
 * @return 0 on successful completion.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Configuration errors are reported as messages naming the file and the accepted values, so
  // print them rather than letting the exception escape main and abort.
  std::optional<InvictaSimParameters> loaded_params;
  std::shared_ptr<InvictaSim> simulator;
  std::shared_ptr<InvictaSimInputAdapter> input_adapter;
  std::shared_ptr<InvictaSimOutputAdapter> output_adapter;
  try {
    loaded_params.emplace();
    simulator = std::make_shared<InvictaSim>(*loaded_params);
    input_adapter = input_adapters_map.at(loaded_params->input_adapter.c_str())(simulator);
    output_adapter = output_adapters_map.at(loaded_params->output_adapter.c_str())(simulator);
  } catch (const std::exception& error) {
    std::cerr << "[invictasim] Startup failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  const InvictaSimParameters& params = *loaded_params;

  std::shared_ptr<TuningEvaluator> tuning_evaluator;

  std::thread simulator_thread([&simulator]() { simulator->run(); });
  std::thread input_thread;
  std::thread output_thread;

  rclcpp::executors::MultiThreadedExecutor executor;

  if (auto ros_input_node = std::dynamic_pointer_cast<rclcpp::Node>(input_adapter)) {
    executor.add_node(ros_input_node);
  } else {
    input_thread = std::thread([&input_adapter]() { input_adapter->run(); });
  }

  if (auto ros_output_node = std::dynamic_pointer_cast<rclcpp::Node>(output_adapter)) {
    executor.add_node(ros_output_node);
  } else {
    output_thread = std::thread([&output_adapter]() { output_adapter->run(); });
  }

  if (params.rosbag_evaluator_enabled) {
    tuning_evaluator = std::make_shared<TuningEvaluator>(simulator, params);
    executor.add_node(tuning_evaluator);
  }

  // Blocks until shutdown, dispatching callbacks across the executor's thread pool. spin_some()
  // in a loop would run every callback on this thread and busy-wait between them.
  executor.spin();

  simulator->stop();
  input_adapter->stop();
  output_adapter->stop();

  if (simulator_thread.joinable()) {
    simulator_thread.join();
  }
  if (input_thread.joinable()) {
    input_thread.join();
  }
  if (output_thread.joinable()) {
    output_thread.join();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
