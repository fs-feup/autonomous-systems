#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "functions/inspection_functions.hpp"

constexpr double WHEELS_STOPPED_THRESHOLD = 0.01;

/**
 * @class InspectionMission
 * @brief Class responsible for the Inspection Mission of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and publishing control commands.
 */
class InspectionMission : public rclcpp::Node {
private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_client;
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_command_publisher;

  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr
      mission_signal_subscription;
  message_filters::Subscriber<custom_interfaces::msg::WheelRPM> rl_rpm_subscription;
  message_filters::Subscriber<custom_interfaces::msg::WheelRPM> rr_rpm_subscription;

  using WSSPolicy =
      message_filters::sync_policies::ApproximateTime<custom_interfaces::msg::WheelRPM,
                                                      custom_interfaces::msg::WheelRPM>;

  std::shared_ptr<message_filters::Synchronizer<WSSPolicy>> sync_;

  rclcpp::TimerBase::SharedPtr mission_end_timer;  // Timer for repetition of end of mission signal
  rclcpp::TimerBase::SharedPtr main_callback_timer;  // Timer for main callback
  rclcpp::Clock clock;
  rclcpp::Time initial_time;  // Ellapsed time in seconds
  InspectionFunctions inspection_object = InspectionFunctions();

  bool go = false;                                 // Flag to start the mission
  common_lib::competition_logic::Mission mission;  // Mission to be executed;

  /**
   * @brief Function for communication of end of mission
   * (or emergency according to the mission)
   */
  void end_of_mission();

  /**
   * @brief Function to end communication of end of mission
   */
  void handle_end_of_mission_response(
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) const;

public:
  /**
   * @brief recieves GoSignal and stores the mission that should be ran
   *
   * @param mission_signal GoSignal
   */
  void mission_decider(custom_interfaces::msg::OperationalStatus::SharedPtr mission_signal);

  /**
   * @brief Function to publish control command while time is less than the defined time limit
   *
   * @param current_rpm rotations of the wheels per minute
   */
  void inspection_script(const custom_interfaces::msg::WheelRPM& current_rlRPM,
                         const custom_interfaces::msg::WheelRPM& current_rrRPM);

  /**
   * @brief Publishes the control command
   *
   * @param throttle throttle to be applied
   * @param steering steering angle to be applied
   */
  void publish_controls(double throttle, double steering) const;

  /**
   * @brief Contruct a new Inspection Mission with constants defined in file
   */
  InspectionMission();
};