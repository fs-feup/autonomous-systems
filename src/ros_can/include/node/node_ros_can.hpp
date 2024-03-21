#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <canlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RosCan : public rclcpp::Node {
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr foo1;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr foo2;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr busStatus;
  rclcpp::TimerBase::SharedPtr timer;  

  // Holds a handle to the CAN channel
  canHandle hnd;
  // Status returned by the Canlib calls
  canStatus stat;

  /**
   * @brief Function to publish (topic) foo1 when a specific can message is recieved
   **/
  void publish_foo1(std_msgs::msg::String::SharedPtr foo1);

  /**
   * @brief Function to publish foo2 to a CAN BUS the
   */
  void foo2_callback(std_msgs::msg::String::SharedPtr foo2);

  /**
   * @brief Function to turn ON and OFF the CAN BUS
   */
  void busStatus_callback(std_msgs::msg::String busStatus);

  /**
   * @brief Function to wait for can messages and create publish them
   */
  void canSniffer();

 public:
  /**
   * @brief Contructor for the RosCan class
   */
  RosCan();
};