#include <chrono>      // Date and time
#include <functional>  // Arithmetic, comparisons, and logical operations
#include <memory>      // Dynamic memory management
#include <string>      // String functions

#include "custom_interfaces/msg/vcu_command.hpp"
#include "rclcpp/rclcpp.hpp"

// chrono_literals handles user-defined time durations (e.g. 500ms)
using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
 public:
  // Constructor creates a node named test_node.
  TestNode() : Node("test_node") {
    // Publisher publishes String messages to a topic named "addison".
    // The size of the queue is 10 messages.
    publisher_ = this->create_publisher<custom_interfaces::msg::VcuCommand>("/cmd", 10);

    // Initialize the timer. The timer_callback function will execute every
    // 500 milliseconds.
    timer_ = this->create_wall_timer(500ms, std::bind(&TestNode::timer_callback, this));
  }

 private:
  // Declaration of the timer_ attribute
  rclcpp::TimerBase::SharedPtr timer_;

  // Declaration of the publisher_ attribute
  rclcpp::Publisher<custom_interfaces::msg::VcuCommand>::SharedPtr publisher_;

  // This method executes every 500 milliseconds
  void timer_callback() {
    // Create a new message of type String
    custom_interfaces::msg::VcuCommand message;

    // random values for testing
    message.status = 0;
    message.direction_request = 1;
    message.estop_request = 1;
    message.steering_angle_request = 10;
    message.axle_speed_request = 500;
    message.axle_torque_request = 15;
    message.brake_press_request = 10;

    printf("Publishing direction request: '%i'", message.direction_request);

    // Publish the message to the topic named "addison"
    publisher_->publish(message);
  }
};