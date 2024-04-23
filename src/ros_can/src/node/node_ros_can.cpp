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
  operationalStatus =
      this->create_publisher<custom_interfaces::msg::OperationalStatus>("operationalStatus", 10);

  rlRPMPub = this->create_publisher<custom_interfaces::msg::WheelRPM>("rlRPM", 10);
  rrRPMPub = this->create_publisher<custom_interfaces::msg::WheelRPM>("rrRPM", 10);

  imuYawAccYPub = this->create_publisher<custom_interfaces::msg::ImuData>("imuYawAccY", 10);
  imuRollAccXPub = this->create_publisher<custom_interfaces::msg::ImuData>("imuRollAccXPub", 10);
  imuPitchAccZPub = this->create_publisher<custom_interfaces::msg::ImuData>("imuPitchAccZPub", 10);

  steeringAngleCubeM =
      this->create_publisher<custom_interfaces::msg::SteeringAngle>("steeringAngleCubeM", 10);
  controlListener = this->create_subscription<fs_msgs::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));
  emergencyListener = this->create_subscription<std_msgs::msg::String>(
      "/as_msgs/emergency", 10,
      std::bind(&RosCan::emergency_callback, this, std::placeholders::_1));  // maybe change type
  missionFinishedListener = this->create_subscription<std_msgs::msg::String>(
      "/as_msgs/mission_finished", 10,
      std::bind(&RosCan::mission_finished_callback, this,
                std::placeholders::_1));  // maybe change type
  // busStatus = this->create_subscription<std_msgs::msg::String>("busStatus", 10,
  // std::bind(&RosCan::busStatus_callback, this, std::placeholders::_1));
  timer =
      this->create_wall_timer(std::chrono::microseconds(500), std::bind(&RosCan::canSniffer, this));
  timerAliveMsg = this->create_wall_timer(std::chrono::milliseconds(100),
                                          std::bind(&RosCan::alive_msg_callback, this));

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
  stat = canBusOn(hnd);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to turn on CAN bus");
  }
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
    RCLCPP_DEBUG(this->get_logger(), "State is Driving: Steering: %f, Throttle: %f",
                 controlCmd->steering, controlCmd->throttle);
    canInitializeLibrary();  // initialize the CAN library again, just in case (could be removed)
    // Prepare the steering message
    long steering_id = STEERING_CUBEM_ID;
    void* steering_requestData = reinterpret_cast<void*>(&controlCmd->steering);
    unsigned int steering_dlc = 8;
    unsigned int flag = 0;

    // Write the steering message to the CAN bus
    stat = canWrite(hnd, steering_id, steering_requestData, steering_dlc, flag);
    if (stat != canOK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write steering to CAN bus");
    }

    // Prepare the throttle message
    long throttle_id = BAMO_RECEIVER;  // TODO: confirm ID
    void* throttle_requestData = reinterpret_cast<void*>(&controlCmd->throttle);
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
/*void RosCan::busStatus_callback(std_msgs::msg::String busStatus) {
  if (busStatus.data == "ON") {
    hnd = canOpenChannel(0, canOPEN_CAN_FD);
    stat = canBusOn(hnd);
  } else if (busStatus.data == "OFF") {
    stat = canBusOff(hnd);
    canClose(hnd);
  }
}*/

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
      if (msg[0] == TEENSY_C1_RR_RPM_CODE) rrRPMPublisher(msg);
      break;
    case STEERING_CUBEM_ID:
      steeringAngleCubeMPublisher(msg);
      break;
    case STEERING_BOSCH_ID:
      steeringAngleBoschPublisher(msg);
    default:
      break;
  }
}

void RosCan::canInterpreterMasterStatus(unsigned char msg[8]) {
  switch (msg[0]) {
    case MASTER_AS_STATE_CODE: {
      if (msg[1] == 3)  // If AS State == Driving
        this->goSignal = 1;
      else
        this->goSignal = 0;
      opStatusPublisher();
      break;
    }
    case MASTER_AS_MISSION_CODE: {
      this->asMission = msg[1];
      opStatusPublisher();
      break;
    }
    case MASTER_RL_RPM_CODE: {
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
  float yawRate = ((msg[1] << 8) | msg[0]) * QUANTIZATION_GYRO;
  float accY = ((msg[5] << 8) | msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.gyro = yawRate;
  message.acc = accY;
  imuYawAccYPub->publish(message);
}

/**
 * @brief Function to publish the Roll rate and acceleration in X
 * @param msg - the CAN msg
 */
void RosCan::imuRollAccXPublisher(unsigned char msg[8]) {
  float rollRate = ((msg[1] << 8) | msg[0]) * QUANTIZATION_GYRO;
  float accX = ((msg[5] << 8) | msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.gyro = rollRate;
  message.acc = accX;
  imuRollAccXPub->publish(message);
}

/**
 * @brief Function to publish the Pitch rate and acceleration in Z
 * @param msg - the CAN msg
 */
void RosCan::imuPitchAccZPublisher(unsigned char msg[8]) {
  float pitchRate = ((msg[1] << 8) | msg[0]) * QUANTIZATION_GYRO;
  float accZ = ((msg[5] << 8) | msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.gyro = pitchRate;
  message.acc = accZ;
  imuPitchAccZPub->publish(message);
}

/**
 * @brief Function to publish the steering angle form steering actuator (CubeMars)
 * @param msg - the CAN msg
 */
void RosCan::steeringAngleCubeMPublisher(unsigned char msg[8]) {
  int angle = (msg[1] << 8) | msg[0];
  auto message = custom_interfaces::msg::SteeringAngle();
  message.steering_angle = angle;
  steeringAngleCubeM->publish(message);
}

/**
 * @brief Function to publish the steering angle form Bosch
 * @param msg - the CAN msg
 */
void RosCan::steeringAngleBoschPublisher(unsigned char msg[8]) {
  // Mask to check the first 3 bits
  char mask = 0b00000111;

  // Extract the first 3 bits
  char firstThreeBits = msg[3] & mask;

  if (firstThreeBits == mask) {
    int angle = (msg[1] << 8) | msg[0];
    int speed = msg[2];
    auto message = custom_interfaces::msg::SteeringAngle();
    message.steering_angle = angle;
    message.steering_speed = speed;
    steeringAngleBosch->publish(message);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid message received from Bosch Speed Sensor");
  }

  int angle = (msg[1] << 8) | msg[0];
  auto message = custom_interfaces::msg::SteeringAngle();
  message.steering_angle = angle;
  steeringAngleCubeM->publish(message);
}

/**
 * @brief Function to publish rrRPM
 * @param msg - the CAN msg
 */
void RosCan::rrRPMPublisher(unsigned char msg[8]) {
  float rrRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.rr_rpm = rrRPM;
  rrRPMPub->publish(message);
}

/**
 * @brief Function to publish rlRPM
 * @param msg - the CAN msg
 */
void RosCan::rlRPMPublisher(unsigned char msg[8]) {
  float rlRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.rl_rpm = rlRPM;
  rlRPMPub->publish(message);
}

void RosCan::alive_msg_callback() {
  long id = 0x400;            // Set the CAN ID
  unsigned char data = 0x41;  // Set the data
  void* msg = &data;
  unsigned int dlc = 1;  // Msg length
  unsigned int flag = 0;

  // Write the emergency message to the CAN bus
  stat = canWrite(hnd, id, msg, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write alive message to CAN bus");
  }
}