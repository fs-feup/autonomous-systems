#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "file_utils/file.hpp"
#include "include/inspection_functions.hpp"
#include "fs_msgs/msg/finished_signal.hpp"

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
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr mission_signal;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr rpm_subscription;
  std::chrono::_V2::system_clock::time_point initial_time;
  InspectionFunctions *inspection_object = new InspectionFunctions();
  std::string mission;

 public:
  /**
   * @brief recieves GoSignal and decides what mission should be used
   * 
   * @param mission_signal GoSignal
   */
  void mission_decider(fs_msgs::msg::GoSignal mission_signal);

  /**
   * @brief runs the mission according to the GoSignal
   * 
   * @param current_rpm rotations of the wheels per minute
   */
  void inspection_general(fs_msgs::msg::WheelStates current_rpm);

  /**
   * @brief Function to publish control command while time is less than 26 seconds
   * 
   * @param current_rpm rotations of the wheels per minute
   */
  void inspection_script(fs_msgs::msg::WheelStates current_rpm);

  /**
   * @brief Function to publish control command while speed is smaller than targer
   * 
   * @param current_rpm rotations of the wheels per minute
   */
  void test_EBS(fs_msgs::msg::WheelStates current_rpm);

  /**
   * @brief Contruct a new Inspection Mission with constants defined in file
   */
  InspectionMission();
};