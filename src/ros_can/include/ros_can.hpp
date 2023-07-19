#ifndef INCLUDE_ROS_CAN_HPP_
#define INCLUDE_ROS_CAN_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <eufs_msgs/msg/can_state.hpp>
#include <eufs_msgs/msg/vehicle_commands_stamped.hpp>
#include <eufs_msgs/msg/wheel_speeds_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>


#include "fs-ai_api.h"  // NOLINT(build/include_subdir)

/**
 * @class CanInterface
 * @brief Interface to communicate with the ADS-DV autonomous car through CAN
 *
 * This program uses the FS-AI library to interface with the ADS-DV and
 * is effectively a ROS wrapper for the library.
 */

class CanInterface : public rclcpp::Node {
 public:
  CanInterface();  // Constructor

  int loop_rate = 100;  // Operational rate of the node

  // Main loop of the node. Gets data from CAN bus. Publishes
  // all info from the car and sends the relevant information to it.
  void loop();

 private:
  // ROS subscribers
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driving_flag_sub_;

  // ROS publishers
  rclcpp::Publisher<eufs_msgs::msg::CanState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_str_;
  rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_pub_;
  rclcpp::Publisher<eufs_msgs::msg::VehicleCommandsStamped>::SharedPtr vehicle_commands_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

  // ROS service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ebs_srv_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Last command message time
  double last_cmd_message_time_;

  // FS-AI API structs to store data
  struct fs_ai_api_vcu2ai_struct vcu2ai_data_;  // Data received from the car
  struct fs_ai_api_ai2vcu_struct ai2vcu_data_;  // Data sent to the car
  struct fs_ai_api_gps_struct gps_data_;        // GPS received from the car
  struct fs_ai_api_imu_struct imu_data_;        // IMU received from the car

  // FS-AI API state enums
  fs_ai_api_mission_status_e mission_status_ = fs_ai_api_mission_status_e::MISSION_NOT_SELECTED;
  fs_ai_api_estop_request_e ebs_state_ = fs_ai_api_estop_request_e::ESTOP_NO;
  fs_ai_api_as_state_e as_state_ = fs_ai_api_as_state_e::AS_OFF;

  // Parameters
  std::string can_interface_ = "can0";
  int can_debug_ = 0;
  int simulate_can_ = 0;

  const float MAX_TORQUE_ = 195;             // Maximum available torque of the car
  const float MAX_RPM_ = 4000;               // Maximum available RPM of the car
  const float MAX_BRAKE_ = 100;              // Maximum available braking of the car
  const float MAX_STEERING_ANGLE_DEG_ = 24;  // Max steering angle (degrees)
  const float WHEEL_RADIUS_ = 0.253;         // Radius of DDT car wheels
  const float WHEELBASE_ = 1.53;             // Wheelbase of the DDT car
  const float TOTAL_MASS_ = 300;             // Total mass of the DDT car in kg

  // Variables
  float steering_ = 0;    // Desired steering
  float torque_ = 0;      // Requested torque
  float braking_ = 0;     // Mechanical braking is zero unless engine braking is insufficient
  float max_dec_ = 10.0;  // Maximum possible deceleration
  float engine_threshold_ = -5.0;  // Maximum engine braking acceleration
  float rpm_limit_ = MAX_RPM_;     // RPM limit on the car
  float rpm_request_ = 0.0;        // Actual rpm request sent to the car
  double cmd_timeout_ = 0.5;       // Timeout for control command

  bool mission_complete_ = false;
  bool driving_flag_ = false;

  /**
   * Handles the handshake bits and returns the one to be sent back to the car
   * @param data received from the CAN bus
   */
  fs_ai_api_handshake_send_bit_e getHandshake(const fs_ai_api_vcu2ai_struct data);

  /**
   * Process the mission status to be sent to the car.
   * @param data received from the CAN bus
   */
  fs_ai_api_mission_status_e getMissionStatus(const fs_ai_api_vcu2ai_struct data);

  /**
   * Derive direction request to send to car. Forward if in
   * AS_DRIVING state and driving flag is true, Neutral otherwise.
   * @param data received from the CAN bus
   */
  fs_ai_api_direction_request_e getDirectionReq(const fs_ai_api_vcu2ai_struct data);

  /**
   * Sets torque/brake and steering commands for the car. Converts
   * them to the appropriate values and checks for limits.
   * @param msg AckermannDriveStamped message from the software system
   */
  void commandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  /**
   * Mission flag for when we're done with a mission
   * @param msg flag message
   */
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * Driving flag that indicates when we're ready to move
   * @param msg flag message
   */
  void drivingFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * Creates a eufs_msgs/VehicleCommandsStamped message which contains the same
   * information that is sent to the car (e.g torque and steering commands)
   */
  eufs_msgs::msg::VehicleCommandsStamped makeVehicleCommandsMessage();

  /**
   * Creates a eufs_msgs/WheelSpeedStamped message. Converts raw values
   * to SI units and the appropriate convention we are using.
   * @param data received from the CAN bus
   */
  eufs_msgs::msg::WheelSpeedsStamped makeWsMessage(const fs_ai_api_vcu2ai_struct data);

  /**
   * Creates a geometry_msgs/TwistWithCovarianceStamped message from wheel speeds
   * @param data received from the CAN bus
   */
  geometry_msgs::msg::TwistWithCovarianceStamped makeTwistMessage(
      const fs_ai_api_vcu2ai_struct data);

  /**
   * Creates an IMU message.
   * @param data received from the CAN bus
   */
  sensor_msgs::msg::Imu makeImuMessage(const fs_ai_api_imu_struct &data);

  /**
   * Creates a GPS message.
   * @param data received from the CAN bus
   */
  sensor_msgs::msg::NavSatFix makeGpsMessage(const fs_ai_api_gps_struct &data);

  /**
   * Creates a eufs_msgs/CanState message using the internal state of the car.
   * @param data received from the CAN bus
   */
  eufs_msgs::msg::CanState makeStateMessage(const fs_ai_api_vcu2ai_struct &data);

  /**
   * Creates an analogous message to eufs_msgs/CanState but in string format for easy reading.
   * @param state of the car
   */
  std_msgs::msg::String
    makeStateString(eufs_msgs::msg::CanState &state);  // NOLINT(runtime/references)

  /**
   * Warns if a values has exceeded it's limits and returns its trunctuated value
   * @param value to be checked
   * @param maximum allowed value
   * @param meaning of the value. used for printing the warning
   */
  float checkAndTrunc(const float val, const float max_val, const std::string type,
                      bool trunc_at_zero = true);
  int checkAndTrunc(const int val, const int max_val, std::string type, bool trunc_at_zero = true);

  /**
   * Sets the Emergency Brake to true to stop the car
   */
  bool requestEBS(std_srvs::srv::Trigger::Request::SharedPtr request,
                  std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * Check if the control message has timed out
   */
  void checkTimeout();
};

#endif  // INCLUDE_ROS_CAN_HPP_
