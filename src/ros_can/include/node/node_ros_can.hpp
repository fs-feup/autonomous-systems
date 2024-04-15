#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "custom_interfaces/msg/imu_data.hpp"
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
 * ID used for:
 * Steering angle
 */
#define STEERING_ID 0x700

/*
 * ID used from the IMU for:
 * yaw rate
 * acceleration in y
 */
#define IMU_YAW_RATE_ACC_Y_ID 0x174

/*
 * ID used from the IMU for:
 * roll rate
 * acceleration in x
 */
#define IMU_ROLL_RATE_ACC_X_ID 0x178

/*
 * ID used from the IMU for:
 * pitch rate
 * acceleration in z
 */
#define IMU_PITCH_RATE_ACC_Z_ID 0x17C


/*
 * Quantization for the acceleration
 * g/digit
 */
#define QUANTIZATION_ACC 0.0001274

/*
 * Quantization for the gyro
 * degree/s/digit
 */
#define QUANTIZATION_GYRO 0.005

#define THROTTLE_UPPER_LIMIT 100
#define THROTTLE_LOWER_LIMIT 0
#define STEERING_UPPER_LIMIT 100
#define STEERING_LOWER_LIMIT 0

class RosCan : public rclcpp::Node {
 private:
  enum class State { AS_Manual, AS_Off, AS_Ready, AS_Driving, AS_Finished, AS_Emergency };
  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr operationalStatus;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr wheelRPM;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr steeringAngle;

  // Enum to hold the state of the AS

  State currentState = State::AS_Off;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr busStatus;
  rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr controlListener;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergencyListener;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr missionFinishedListener;
  rclcpp::TimerBase::SharedPtr timer;

  // Holds a handle to the CAN channel
  canHandle hnd;
  // Status returned by the Canlib calls
  canStatus stat;

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
   * @param id - the CAN msg id
   * @param msg - the CAN msg
   * @param dlc - the CAN msg length
   * @param flag - the CAN msg flag - see kvaser documentation for more info
   * @param time - the CAN msg time stamp
   */
  void canInterpreter(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag,
                      unsigned long time);

  /**
   * @brief Function to publish the Operational Status
   */
  void opStatusPublisher();

  /**
   * @brief Function to publish the Yaw rate and Acceleration in Y
   * @param msg - the CAN msg
   */
  void imuYawAccYPublisher(unsigned char msg[8]);

  /**
   * @brief Function to publish the Roll rate and Acceleration in X
   * @param msg - the CAN msg
   */
  void imuRollAccXPublisher(unsigned char msg[8]);

  /**
   * @brief Function to publish the Pitch rate and Acceleration in Z
   * @param msg - the CAN msg
   */
  void imuPitchAccZPublisher(unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle
   * @param angleLSB - the steering angle least significant byte
   * @param angleMSB - the steering angle most significant byte
   */
  void steeringAnglePublisher(unsigned char angleLSB, unsigned char angleMSB);

  /**
   * @brief Function to publish the rear right rpm
   * @param msg - the CAN msg
   */
  void rrRPMPublisher(unsigned char msg[8]);

/**
   * @brief Function to publish the rear left rpm
   * @param msg - the CAN msg
   */
  void rlRPMPublisher(unsigned char msg[8]);

  


  /**
   * @brief Function to interpret the master status CAN msg
   * @param msg - the CAN msg
   */
  void canInterpreterMasterStatus(unsigned char msg[8]);

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
