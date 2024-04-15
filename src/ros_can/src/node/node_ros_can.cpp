#include "node/node_ros_can.hpp"

#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

RosCan::RosCan() : Node("node_ros_can") {
  operationalStatus =
      this->create_publisher<custom_interfaces::msg::OperationalStatus>("operationalStatus", 10);
  wheelRPM = this->create_publisher<custom_interfaces::msg::WheelRPM>("wheelRPM", 10);
  imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  steeringAngle =
      this->create_publisher<custom_interfaces::msg::SteeringAngle>("steeringAngle", 10);
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
    canInterpreter(id, msg, dlc, flag, time);

    stat = canRead(hnd, &id, &msg, &dlc, &flag, &time);
  }
}

void RosCan::canInterpreter(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag,
                            unsigned long time) {
  switch (id) {
    case MASTER_STATUS:
      canInterpreterMasterStatus(msg);
      break;
    case IMU_YAW_RATE_ACC_Y_ID:
      imuYawAccYPublisher(msg);
      break;
    case IMU_ROLL_RATE_ACC_X_ID:
      imuRollAccXPublisher(msg);
      break;
    case IMU_PITCH_RATE_ACC_Z_ID:
      imuPitchAccZPublisher(msg);
      break;
    case TEENSY_C1:
      if (msg[0] == 0x11)
        rrRPMPublisher(msg);
      break;
    case STEERING_ID:
      steeringAnglePublisher(msg[1], msg[2]);
      break;

    default:
      break;
  }
}

void RosCan::canInterpreterMasterStatus(unsigned char msg[8]) {
  switch (msg[0]) {
    case 0x31:  // Current AS State
    {
      if (msg[1] == 3)  // If AS State == Driving
        this->goSignal = 1;
      else
        this->goSignal = 0;
      opStatusPublisher();
      break;
    }
    case 0x32:  // Current AS Mission
    {
      this->asMission = msg[1];
      opStatusPublisher();
      break;
    }
    case 0x33:  // Left Wheel RPM
    {
      rlRPMPublisher(msg);
      break;
    }
    default:
      break;
  }
}


/**
 * @brief Function to publish the Operational Status
 */
void RosCan::opStatusPublisher() {
  auto message = custom_interfaces::msg::OperationalStatus();
  message.go_signal = goSignal;
  message.as_mission = asMission;
  operationalStatus->publish(message);
}




/**
 * @brief Function to publish the Yaw rate and acceleration in y
 * @param msg - the CAN msg
 */
void RosCan::imuYawAccYPublisher(unsigned char msg[8]) {
  float yawRate = ((msg[1] << 8) | msg[0] ) * QUANTIZATION_GYRO;
  float accY = ((msg[5] << 8) | msg[4] ) * QUANTIZATION_ACC;
  int msgId = 1;
  
}

/**
 * @brief Function to publish the Roll rate and acceleration in X
 * @param msg - the CAN msg
 */
void RosCan::imuRollAccXPublisher(unsigned char msg[8]) {
  float rollRate = ((msg[1] << 8) | msg[0] ) * QUANTIZATION_GYRO;
  float accX = ((msg[5] << 8) | msg[4] ) * QUANTIZATION_ACC;
  int msgId = 2;
}


/**
 * @brief Function to publish the Pitch rate and acceleration in Z
 * @param msg - the CAN msg
 */
void RosCan::imuPitchAccZPublisher(unsigned char msg[8]) {
  float pitchRate = ((msg[1] << 8) | msg[0] ) * QUANTIZATION_GYRO;
  float accZ = ((msg[5] << 8) | msg[4] ) * QUANTIZATION_ACC;
  int msgId = 3;

}


/**
 * @brief Function to publish the steering angle
 * @param angleLSB - the steering angle least significant byte
 * @param angleMSB - the steering angle most significant byte
 */
void RosCan::steeringAnglePublisher(unsigned char angleLSB, unsigned char angleMSB) {
  // Publish the steering angle to a topic
  int angle = (angleMSB << 8) | angleLSB;
  auto message = custom_interfaces::msg::SteeringAngle();
  message.steering_angle = angle;
  steeringAngle->publish(message);
}


/**
 * @brief Function to update the wheel rpm
 * @return the wheel rpm
 * @param msg - the CAN msg
 */
float RosCan::updatewheelRPM(unsigned char msg[8]) {
  int32_t RPMAux = (msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1];
  return RPMAux / 100.0f;
}

