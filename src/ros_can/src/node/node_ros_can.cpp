#include "node/node_ros_can.hpp"

#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "fs_msgs/msg/control_command.hpp"
// #include <time.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

RosCan::RosCan() : Node("node_ros_can") {
  foo1 = this->create_publisher<std_msgs::msg::String>("foo1", 10);
  // foo2 = this->create_subscription<std_msgs::msg::String>(
  //     "foo2", 10, std::bind(&RosCan::foo2_callback, this, std::placeholders::_1));
  foo2 = this->create_subscription<fs_msgs::msg::ControlCommand>(
      "foo2", 10, std::bind(&RosCan::foo2_callback, this, std::placeholders::_1));
  busStatus = this->create_subscription<std_msgs::msg::String>(
      "busStatus", 10, std::bind(&RosCan::busStatus_callback, this, std::placeholders::_1));
  timer =
      this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RosCan::canSniffer, this));
  // initialize the CAN library
  canInitializeLibrary();
  // A channel to a CAN circuit is opened. The channel depend on the hardware
  hnd = canOpenChannel(1, canOPEN_CAN_FD);
  // Setup CAN parameters for the channel
  stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);  // check these values later
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN parameters");
  }
  // There are different types of controllers, this is the default
  stat = canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN controller");
  }
  canSniffer();
}

void RosCan::foo2_callback(fs_msgs::msg::ControlCommand::SharedPtr foo2) {
  canInitializeLibrary();  
  // Prepare the steering message
  long steering_id = steering_id;//TODO: check ID
  void* steering_msgData = (void*)&foo2->steering;
  unsigned int steering_dlc = 8;
  unsigned int flag = 0;

  // Write the steering message to the CAN bus
  stat = canWrite(hnd, steering_id, steering_msgData, steering_dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write steering to CAN bus");
  }

  // Prepare the throttle message
  long throttle_id = 0x201;//TODO: check ID
  void* throttle_msgData = (void*)&foo2->throttle;
  unsigned int throttle_dlc = 8;

  // Write the throttle message to the CAN bus
  stat = canWrite(hnd, throttle_id, throttle_msgData, throttle_dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write throttle to CAN bus");
  }
}

/**
 * @brief Function to turn ON and OFF the CAN BUS
 */
void RosCan::busStatus_callback(std_msgs::msg::String busStatus) {
  if (busStatus.data == "ON") {
    hnd = canOpenChannel(0, canOPEN_CAN_FD);
    stat = canBusOn(hnd);
  } else if (busStatus.data == "OFF") {
    stat = canBusOff(hnd);
    canClose(hnd);
  }
}

void RosCan::canSniffer() {
  /**
   * @todo Implement function to read and create topics from can messages
   **/
  long id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;

  stat = canRead(hnd, &id, &msg, &dlc, &flag, &time);
  if (stat == canOK) {
    // Convert the CAN message to a ROS message and publish it
    auto message = std_msgs::msg::String();
    message.data = std::string(reinterpret_cast<char*>(msg), dlc);
    foo1->publish(message);
  }
}