#include "can/can.hpp"
#include "test_node.cpp"

using namespace std::chrono_literals;


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Comment if testing
  rclcpp::spin(std::make_shared<Can>());
  
  // Uncomment if testing
  /*
  // Create instances of the nodes
  auto test_node = std::make_shared<TestNode>();
  auto can_node = std::make_shared<Can>();

  // Create a multi-threaded executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Add nodes to the executor
  executor->add_node(test_node);
  executor->add_node(can_node);

  // Spin the nodes using the executor
  executor->spin();
  */
  
  rclcpp::shutdown();

  return 0;
}
