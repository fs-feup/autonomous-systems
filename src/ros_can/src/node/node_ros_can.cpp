#include "node/node_ros_can.hpp"

#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
// #include <time.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

RosCan::RosCan() : Node("node_ros_can") {
  asState = this->create_publisher<std_msgs::msg::Int32>("asState", 10);
  asMission = this->create_publisher<std_msgs::msg::Int32>("asMission", 10);
  leftWheel = this->create_publisher<std_msgs::msg::Float32>("leftWheel", 10);
  rightWheel = this->create_publisher<std_msgs::msg::Float32>("rightWheel", 10);
  imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  steeringAngle = this->create_publisher<std_msgs::msg::Float32>("steeringAngle", 10);
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

/**
 * @brief Function cyclically reads all CAN msg from buffer
 */
void RosCan::canSniffer() {
  long id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;

  stat = canRead(hnd, &id, &msg, &dlc, &flag, &time);
  while (stat == canOK) {
    // Convert the CAN message to a ROS message and publish it
    canInterperter(id, msg, dlc, flag, time);

    stat = canRead(hnd, &id, &msg, &dlc, &flag, &time);
  }
}

void RosCan::canInterperter(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag,
                    unsigned long time) {
  switch (id) {
    case MASTER_STATUS:
      switch (msg[0]) {
        case 0x31:  // Current AS State
          asStatePublisher(msg[1]);
          break;
        case 0x32:  // Current AS Mission
          asMissionPublisher(msg[1]);
        case 0x33:  // Left Wheel RPM
          leftWheelPublisher(msg[1], msg[2]);
          break;
        default:
          break;
      }
      break;

    case MASTER_IMU:
      imuPublisher(msg);
      break;
    case TEENSY_C1:
      if (msg[0] == 0x11) rightWheelPublisher(msg[1], msg[2]);
      break;
    case STEERING_ID:
      steeringAnglePublisher(msg[1], msg[2]);
      break;

    default:
      break;
  }
}

/**
 * @brief Function to publish the AS state
 */
void RosCan::asStatePublisher(unsigned char state) {
  // Publish the current AS state to a topic
  if (state >= 0 && state <= 5) {
    auto message = std_msgs::msg::Int32();
    message.data = state;
    asState->publish(message);
  }
}

/**
 * @brief Function to publish the AS mission
 */
void RosCan::asMissionPublisher(unsigned char mission) {
  // Publish the current AS mission to a topic
  if (mission >= 0 && mission <= 6) {
    auto message = std_msgs::msg::Int32();
    message.data = mission;
    asMission->publish(message);
  }
}

/**
 * @brief Function to publish the left wheel rpm
 */
void RosCan::leftWheelPublisher(unsigned char rpmLSB, unsigned char rpmMSB) {
  // Publish the left wheel rpm to a topic
  float rpm = (rpmMSB << 8) | rpmLSB;
  auto message = std_msgs::msg::Float32();
  message.data = rpm;
  leftWheel->publish(message);
}

/**
 * @brief Function to publish the right wheel rpm
 */
void RosCan::rightWheelPublisher(unsigned char rpmLSB, unsigned char rpmMSB) {
  // Publish the right wheel rpm to a topic
  float rpm = (rpmMSB << 8) | rpmLSB;
  auto message = std_msgs::msg::Float32();
  message.data = rpm;
  rightWheel->publish(message);
}

/**
 * @brief Function to publish the imu values
 */
void RosCan::imuPublisher(unsigned char msg[8]) {
  // Publish the imu values to a topic
  auto message = sensor_msgs::msg::Imu();
  message.linear_acceleration.x = (msg[2] << 8) | msg[1];
  message.linear_acceleration.y = (msg[4] << 8) | msg[3];
  message.angular_velocity.z = (msg[6] << 8) | msg[5];
  imu->publish(message);
}

/**
* @brief Function to publish the steering angle
*/
void RosCan::steeringAnglePublisher(unsigned char angleLSB, unsigned char angleMSB) {
  // Publish the steering angle to a topic
  float angle = (angleMSB << 8) | angleLSB;
  auto message = std_msgs::msg::Float32();
  message.data = angle;
  steeringAngle->publish(message);
}