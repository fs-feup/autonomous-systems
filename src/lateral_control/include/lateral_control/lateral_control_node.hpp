
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LateralControl : public rclcpp::Node {
 private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ebs_status_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr steering_command_pub;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_velocity;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_pos;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr path_subscription;

  void publish_steeringcommand(custom_interfaces::msg::ConeArray path);
  void publish_latctrl_ebs_status();

 public:
  LateralControl();
};