#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"

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

class RosCan : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr operationalStatus;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr wheelRPM;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr steeringAngle;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr busStatus;
  rclcpp::TimerBase::SharedPtr timer;

  // Holds a handle to the CAN channel
  canHandle hnd;
  // Status returned by the Canlib calls
  canStatus stat;

  /*
  wheelRPMsync is 0, we didnt receive any value
  wheelRPMsync is 1, we received one wheel rpm
  */ 
  bool wheelRPMsync=0;

  float leftWheelRPM;
  float rightWheelRPM;

  /*
  goSignal:
  0 - Stop
  1 - Go
  */
  bool goSignal = 0;

  
  /*Current Mission:
  0 - Manual
  1 - Acceleration
  2 - Skidpad
  3 - Autocross
  4 - Trackdrive
  5 - EBS_Test
  6 - Inspection*/
  int asMission;

  


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
   * @brief Function to publish the Operational Status
   */
  void opStatusPublisher();

  /**
   * @brief Function to publish the left wheel rpm
   */
  void WheelRPMPublisher();

  /**
   * @brief Function to publish the imu values
   */
  void imuPublisher(unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle
   */
  void steeringAnglePublisher(unsigned char angleLSB, unsigned char angleMSB);

 public:
  /**
   * @brief Contructor for the RosCan class
   */
  RosCan();
};
