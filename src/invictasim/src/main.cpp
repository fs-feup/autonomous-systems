#include <chrono>
#include <thread>

#include "config/config.hpp"
#include "io/input/map.hpp"
#include "io/output/map.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "simulator/invictasim.hpp"

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

  InvictaSimParameters params = InvictaSimParameters();
  auto simulator = std::make_shared<InvictaSim>(params);

  auto input_adapter = input_adapters_map.at(params.input_adapter.c_str())(simulator);
  auto output_adapter = output_adapters_map.at(params.output_adapter.c_str())(simulator);

  std::thread simulator_thread([&simulator]() { simulator->run(); });
  std::thread input_thread([&input_adapter]() { input_adapter->run(); });
  std::thread output_thread([&output_adapter]() { output_adapter->run(); });

  auto ros_input_node = std::dynamic_pointer_cast<rclcpp::Node>(input_adapter);
  auto ros_output_node = std::dynamic_pointer_cast<rclcpp::Node>(output_adapter);

  rclcpp::executors::SingleThreadedExecutor executor;
  if (ros_input_node) {
    executor.add_node(ros_input_node);
  }
  if (ros_output_node) {
    executor.add_node(ros_output_node);
  }

  while (rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

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
