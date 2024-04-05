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
  controlListener = this->create_subscription<fs_msgs::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));
  emergencyListener = this->create_subscription<std_msgs::msg::String>(
      "/as_msgs/emergency", 10, std::bind(&RosCan::emergency_callback, this, std::placeholders::_1));//maybe change type
  missionFinishedListener = this->create_subscription<std_msgs::msg::String>(
      "/as_msgs/mission_finished", 10,
      std::bind(&RosCan::mission_finished_callback, this, std::placeholders::_1));//maybe change type
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
  canSniffer();
}

void RosCan::control_callback(fs_msgs::msg::ControlCommand::SharedPtr controlCmd) {
  // Check if the steering value is within the limits
  if (controlCmd->steering < STEERING_LOWER_LIMIT || controlCmd->steering > STEERING_UPPER_LIMIT) {
    RCLCPP_ERROR(this->get_logger(), "Steering value out of range");
    return;
  }
  // Check if the throttle value is within the limits
  if (controlCmd->throttle < THROTTLE_LOWER_LIMIT || controlCmd->throttle > THROTTLE_UPPER_LIMIT) {
    RCLCPP_ERROR(this->get_logger(), "Throttle value out of range");
    return;
  } 
  if (currentState == State::AS_Driving) {
    RCLCPP_DEBUG(this->get_logger(), "State is Driving: Steering: %f, Throttle: %f", controlCmd->steering,
                 controlCmd->throttle);
    canInitializeLibrary();  // initialize the CAN library again, just in case (could be removed)
    // Prepare the steering message
    long steering_id = STEERING_ID;  // TODO: confirm ID
    void* steering_requestData = (void*)&controlCmd->steering;
    unsigned int steering_dlc = 8;
    unsigned int flag = 0;

    // Write the steering message to the CAN bus
    stat = canWrite(hnd, steering_id, steering_requestData, steering_dlc, flag);
    if (stat != canOK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write steering to CAN bus");
    }

    // Prepare the throttle message
    long throttle_id = 0x201;  // TODO: confirm ID
    void* throttle_requestData = (void*)&foo2->throttle;
    unsigned int throttle_dlc = 8;

    // Write the throttle message to the CAN bus
    stat = canWrite(hnd, throttle_id, throttle_requestData, throttle_dlc, flag);
    if (stat != canOK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write throttle to CAN bus");
    }
  }
}

/**
 * @brief Function to handle the emergency message
 */
void RosCan::emergency_callback(std_msgs::msg::String::SharedPtr msg) {
  // Prepare the emergency message
  long id = 0x400;            // Set the CAN ID
  unsigned char data = 0x43;  // Set the data
  void* requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;

  // Write the emergency message to the CAN bus
  stat = canWrite(hnd, id, requestData, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

/**
 * @brief Function to handle the mission finished message
 */
void RosCan::mission_finished_callback(std_msgs::msg::String::SharedPtr msg) {
  // Prepare the emergency message
  long id = 0x400;            // Set the CAN ID
  unsigned char data = 0x42;  // Set the data
  void* requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;

  // Write the emergency message to the CAN bus
  stat = canWrite(hnd, id, requestData, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
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
          currentState = static_cast<State>(
              msg[1]);  // TODO: check if static casting is the best solution here
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