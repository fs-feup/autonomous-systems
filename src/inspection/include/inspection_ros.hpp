#ifndef INSPECTION_MISSION_HPP
#define INSPECTION_MISSION_HPP

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "custom_interfaces/msg/emergency.hpp"
#include "include/inspection_functions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include <message_filters/subscriber.h>
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

/**
 * @class InspectionMission
 * @brief Class responsible for the Inspection Mission of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and publishing control commands.
 */
class InspectionMission : public rclcpp::Node {
 private:
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr finish_publisher;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_command_publisher;
  rclcpp::Publisher<custom_interfaces::msg::Emergency>::SharedPtr emergency_publisher;
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr mission_signal;

  message_filters::Subscriber<custom_interfaces::msg::WheelRPM> rlRPM_subscription;
  message_filters::Subscriber<custom_interfaces::msg::WheelRPM> rrRPM_subscription;

  using WSSPolicy = message_filters::sync_policies::ApproximateTime<custom_interfaces::msg::WheelRPM, custom_interfaces::msg::WheelRPM>;

  std::shared_ptr<message_filters::Synchronizer<WSSPolicy>> sync_;

  std::chrono::_V2::system_clock::time_point initial_time;
  InspectionFunctions *inspection_object;
  std::string mission;

 public:
  /**
   * @brief recieves GoSignal and stores the mission that should be ran
   *
   * @param mission_signal GoSignal
   */
  void mission_decider(fs_msgs::msg::GoSignal mission_signal);

  /**
   * @brief Function to publish control command while time is less than the defined time limit
   *
   * @param current_rpm rotations of the wheels per minute
   */
  void inspection_script(custom_interfaces::msg::WheelRPM current_rlRPM, custom_interfaces::msg::WheelRPM current_rrRPM);

  /**
   * @brief Publishes the control command
   *
   * @param torque torque to be applied
   * @param steering steering angle to be applied
   */
  void publish_controls(double torque, double steering);

  /**
   * @brief Contruct a new Inspection Mission with constants defined in file
   */
  InspectionMission();

  void callback(std_msgs::msg::String str1, std_msgs::msg::String str2);
};

#endif  // INSPECTION_MISSION_HPP