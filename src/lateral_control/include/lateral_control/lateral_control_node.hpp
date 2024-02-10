
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class LateralControl
 * @brief Class responsible for the Lateral control of the car
 *
 * Subscription: Path (current pos and current velocity)
 * Publisher: EBS status, Steering command
 */

class LateralControl : public rclcpp::Node {
 private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ebs_status_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr steering_command_pub;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_velocity;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_pos;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr path_subscription;

  /**
   * @brief Publish Steering command when a new path is received
   *
   * @param path - the new path
   * @return void
   */
  void publish_steeringcommand(custom_interfaces::msg::ConeArray path);

  /**
   * @brief Publish the EBS Status
   *
   * @param tbd
   * @return void
   */
  void publish_latctrl_ebs_status();

 public:
  /**
   * @brief Class Constructor
   */
  LateralControl();
};