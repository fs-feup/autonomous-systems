#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

/*
 * ID used for:
 * Current AS Mission
 * Current AS State
 * left wheel rpm
 */
#define MASTER_STATUS 0x300

/*
 * ID used for:
 * Left wheel rpm
 */
#define TEENSY_C1 0x123

/*
 * ID used for imu values
 */
#define MASTER_IMU 0x501

/*
 * ID used for:
 * Steering angle
 */
#define STEERING_ID 0x700

// define throttle and steering upper and lower limit values
#define THROTTLE_UPPER_LIMIT 100
#define THROTTLE_LOWER_LIMIT 0
#define STEERING_UPPER_LIMIT 100
#define STEERING_LOWER_LIMIT 0

class RosCan : public rclcpp::Node {
 private:
  // Enum to hold the state of the AS
  enum class State { AS_Manual, AS_Off, AS_Ready, AS_Driving, AS_Finished, AS_Emergency };

  State currentState = State::AS_Off;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr asState;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr asMission;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftWheel;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightWheel;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steeringAngle;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr busStatus;
  rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr controlListener;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergencyListener;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr missionFinishedListener;
  rclcpp::TimerBase::SharedPtr timer;

  // Holds a handle to the CAN channel
  canHandle hnd;
  // Status returned by the Canlib calls
  canStatus stat;

  /**
   * @brief Function to turn ON and OFF the CAN BUS
   */
  void busStatus_callback(std_msgs::msg::String busStatus);

  /**
   * @brief Function cyclically reads all CAN msg from buffer
   */
  void canSniffer();

  /**
   * @brief Function to interpret the CAN msg
   */
  void canInterperter(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag,
                      unsigned long time);

  /**
   * @brief Function to publish the AS state
   */
  void asStatePublisher(unsigned char state);

  /**
   * @brief Function to publish the AS mission
   */
  void asMissionPublisher(unsigned char mission);

  /**
   * @brief Function to publish the left wheel rpm
   */
  void leftWheelPublisher(unsigned char rpmLSB, unsigned char rpmMSB);

  /**
   * @brief Function to publish the right wheel rpm
   */
  void rightWheelPublisher(unsigned char rpmLSB, unsigned char rpmMSB);

  /**
   * @brief Function to publish the imu values
   */
  void imuPublisher(unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle
   */
  void steeringAnglePublisher(unsigned char angleLSB, unsigned char angleMSB);

  /**
   * @brief Function to handle the emergency message
   */
  void emergency_callback(std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Function to handle the mission finished message
   */
  void mission_finished_callback(std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Function to handle the control command message
   */
  void control_callback(fs_msgs::msg::ControlCommand::SharedPtr msg);

 public:
  /**
   * @brief Contructor for the RosCan class
   */
  RosCan();
};
