#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LongitudinalControl : public rclcpp::Node {
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      result;  // variable previously named publisher_
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      current_velcoity;  // variable previously named subscriber_
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desired_velcoity;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr path_subscription;
  double velocity;

  void publish_torque(custom_interfaces::msg::ConeArray path);

  void velocity_estimation_callback(std_msgs::msg::String velocity);

 public:
  LongitudinalControl();
};