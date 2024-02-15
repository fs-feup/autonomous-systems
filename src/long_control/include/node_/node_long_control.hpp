#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter/adapter.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class LongitudinalControl
 * @brief Class responsible for the longitudinal control of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and ideal path topics, and publishing torque (or other output to the actuators).
 */
class LongitudinalControl : public rclcpp::Node {
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_velcoity;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr path_subscription;
  double velocity;
  Adapter *adapter;

  /**
   * @brief Function to publish the desired output (provisionally torque)
   * when a new map is recieved
   */
  void publish_torque(custom_interfaces::msg::ConeArray path);

  /**
   * @brief Function to hold the value of the velocity when
   * new velocity data is recieved
   */
  void velocity_estimation_callback(std_msgs::msg::String velocity);

 public:
  /**
   * @brief Contructor for the LongitudinalControl class
   */
  LongitudinalControl();
};