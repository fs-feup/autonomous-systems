#include <canlib.h>
#include <gtest/gtest_prod.h>

#include <chrono>
#include <functional>
#include <memory>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "canlib_wrappers/ican_lib_wrapper.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/imu_data.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "custom_interfaces/msg/imu_acceleration.hpp"
#include "custom_interfaces/msg/yaw_pitch_roll.hpp"
#include "custom_interfaces/msg/hydraulic_line_pressure.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class RosCan
 * @brief Provides a translation interface between ROS and CAN communication.
 * 
 * The `RosCan` class translates between CAN messages and ROS topics/services. It handles CAN communication
 * using the `ICanLibWrapper` and publishes data to ROS topics. Additionally, it subscribes to ROS topics
 * and services, forwarding commands and status updates to the CAN bus.
 */
class RosCan : public rclcpp::Node {
private:
  /**
   * @enum State
   * @brief Represents the various operational states of the vehicle.
   */
  enum class State { AS_MANUAL, AS_OFF, AS_READY, AS_DRIVING, AS_FINISHED, AS_EMERGENCY };


  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr operational_status_; ///< Publisher for operational status
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr rl_rpm_pub_; ///< Publisher for rear left wheel RPM
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr rr_rpm_pub_; ///< Publisher for rear right wheel RPM
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr motor_rpm_pub_; ///< Publisher for motor RPM
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr
      bosch_steering_angle_publisher_; ///< Publisher for Bosch steering angle
  rclcpp::Publisher<custom_interfaces::msg::HydraulicLinePressure>::SharedPtr
      hydraulic_line_pressure_publisher_; ///< Publisher for hydraulic line pressure


  // IMU Data Publishers
  rclcpp::Publisher<custom_interfaces::msg::ImuAcceleration>::SharedPtr imu_acc_pub_; ///< Publisher for IMU acceleration data
  rclcpp::Publisher<custom_interfaces::msg::YawPitchRoll>::SharedPtr imu_angular_velocity_pub_; ///< Publisher for IMU angular velocity data

  // Enum to hold the state of the AS
  State current_state_ = State::AS_OFF; ///< Current operational state of the vehicle
  int battery_voltage_ = 0; ///< Battery voltage in volts, in Bamocar scale
  int motor_speed_ = 0; ///< Motor speed in RPM, in Bamocar scale
  int hydraulic_line_pressure_ = 0; ///< Hydraulic line pressure
  double steering_angle_ = 0.0; ///< Steering angle in radians (steering column)

  std::shared_ptr<ICanLibWrapper> can_lib_wrapper_; ///< Wrapper for CAN library

  // rclcpp::Subscription<std_msgs::msg::String::SharedPtr> busStatus;
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr control_listener_; ///< Subscription for control commands
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_; ///< Service for emergency handling
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_finished_service_;  ///< Service for mission status updates
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic tasks
  rclcpp::TimerBase::SharedPtr timer_alive_msg_; ///< Timer for sending alive messages

  canHandle hnd_; ///< Handle to the CAN channel

  canStatus stat_; ///< Status of the last CANlib call

  bool go_signal_ = false; ///< Flag to control vehicle (false - stop/ true - go)

  /* Current Mission:
    0 - Manual
    1 - Acceleration
    2 - Skidpad
    3 - Autocross
    4 - Trackdrive
    5 - EBS_Test
    6 - Inspection 
  */
  int as_mission_; ///< Current mission index

  bool cubem_configuration_sent_ = false; ///< Flag to check if Cubemars configuration is sent

  double cubem_steering_angle_ = 0.0; ///< Steering angle from Cubemars

  /**
   * @brief Function to turn ON and OFF the CAN BUS
   * @param busStatus - the status of the bus
   
   void busStatus_callback(std_msgs::msg::String busStatus);
  */

  /**
   * @brief Function to cyclically read all CAN messages from the buffer.
   * This function continuously monitors the CAN bus and processes incoming CAN messages.
   */
  void can_sniffer();

  /**
   * @brief Parses and interprets a CAN message and processes it based on its ID and content.
   * @param id CAN message ID
   * @param msg CAN message data
   * @param dlc Data length code
   * @param flag CAN message flags - see kvaser documentation for more info
   * @param time Timestamp of the CAN message
   */
  void can_interpreter(long id, const unsigned char msg[8], unsigned int, unsigned int,
                       unsigned long);

  /**
   * @brief Publishes the current operational status to ROS.
   */
  void op_status_publisher();

  /**
   * @brief Receives IMU accelerations from CAN and publishes them to ROS
   * @param msg CAN message data
   */
  void imu_acc_publisher(const unsigned char msg[8]);

  /**
   * @brief Receives IMU orientation from CAN and publishes them to ROS.
   * @param msg CAN message data
   */
  void imu_angular_velocity_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the steering angle from Cubemars steering actuator. 
   * Used only to initially set actuator origin
   * @param msg CAN message data
   */
  void steering_angle_cubem_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the steering angle from Bosch.
   * @param msg CAN message data
   */
  void steering_angle_bosch_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the rear right RPM to ROS.
   * @param msg CAN message data
   */
  void rr_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the rear left RPM to ROS.
   * @param msg CAN message data
   */
  void rl_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Receives hydraulic line pressure from CAN and processes it.
   * Used to check if you can go to Driving
   * @param msg CAN message data
   */
  void hydraulic_line_callback(const unsigned char msg[8]);

  /**
   * @brief Publishes the motor speed, received from the encoder, to ROS.
   * 
   * @param msg CAN message data
   */
  void motor_speed_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the battery voltage, received from BAMOCAR, to ROS.
   * 
   * @param msg CAN message data
   */
  void battery_voltage_callback(const unsigned char msg[8]);

  /**
   * @brief Interprets the BAMOCAR CAN message.
   * 
   * @param msg CAN message data
   */
  void can_interpreter_bamocar(const unsigned char msg[8]);

  /**
   * @brief Interprets the master status CAN message.
   * 
   * @param msg CAN message data
   */
  void can_interpreter_master_status(const unsigned char msg[8]);

  /**
   * @brief Handles the emergency service callback.
   * 
   * @param request Service request
   * @param response Service response
   */
  void emergency_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Handles the mission finished service callback.
   * 
   * @param request Service request
   * @param response Service response
   */
  void mission_finished_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Handles the control command message callback.
   * 
   * @param msg Control command message
   */
  void control_callback(custom_interfaces::msg::ControlCommand::SharedPtr msg);

  /**
   * @brief Sends the steering control command to CAN.
   * 
   * @param steering_angle_command Steering angle in radians
   */
  void send_steering_control(double steering_angle_command);

  /**
   * @brief Sends the throttle control command to CAN.
   * 
   * @param throttle_value_ros Throttle value from ROS
   */
  void send_throttle_control(double throttle_value_ros);

  /**
   * @brief Sends an alive message from the AS CU to the master periodically.
   */
  void alive_msg_callback();

  /**
   * @brief Sets the angle origin of the Cubemars steering actuator.
   */
  void cubem_set_origin();

  /**
   * @brief Sets the current position as the Bosch steering angle origin (permanent)
   * @note This function is not currently used as it is meant to be used only once
   */
  void bosch_steering_angle_set_origin();

public:

  /**
   * @brief Contructor for the RosCan class
   */
  RosCan(std::shared_ptr<ICanLibWrapper> can_lib_wrapper_param);

  friend class RosCanTest;
  FRIEND_TEST(RosCanTest, ControlCallback);
  FRIEND_TEST(RosCanTest, ControlCallbackLowerThrottle);
  FRIEND_TEST(RosCanTest, ControlCallbackHigherThrottle);
  FRIEND_TEST(RosCanTest, ControlCallbackLowerSteering);
  FRIEND_TEST(RosCanTest, ControlCallbackHigherSteering);
  FRIEND_TEST(RosCanTest, PublishControlCallback);
  FRIEND_TEST(RosCanTest, TestImuAccPublisher);
  FRIEND_TEST(RosCanTest, TestImuGyroPublisher);
  FRIEND_TEST(RosCanTest, TestCanInterpreterMasterStatusMission);
  FRIEND_TEST(RosCanTest, TestCanInterpreter_TEENSY_C1_RR_RPM_CODE);
  FRIEND_TEST(RosCanTest, TestOutOfRangeUpperSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestOutOfRangeLowerSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestOutOfRangeSingleSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestCarStateMustBeDriving);
  FRIEND_TEST(RosCanTest, TestAliveMsgCallback);
};