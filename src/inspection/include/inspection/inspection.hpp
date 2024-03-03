#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "fs_msgs/msg/wheel_states.hpp"

/**
 * @class InspectionMission
 * @brief Class responsible for the Inspection Mission of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and ideal path topics, and publishing torque (or other output to the actuators).
 */
class InspectionMission : public rclcpp::Node {
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flag_publisher;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_command_publisher;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr rpm_subscription;
  double max_angle  = 11.0/21.0; // in radians
  double ideal_speed = 1.0; // in meters per second
  double turning_period = 4.0; // in seconds
  double wheel_radius = 2.54/10.0; // in meters
  double gain = 0.1;
  std::chrono::_V2::system_clock::time_point initial_time = std::chrono::system_clock::now();

 public:
  /**
   * @brief Function to publish the desired output (provisionally torque)
   * when a new map is recieved
   */
  void publish_torque_steer(fs_msgs::msg::WheelStates current_rpm);


  /**
   * @brief get the steering angle according to time
   * 
   * @param time in seconds
   * @return steering angle in radians
   */
  double calculate_steering(double time);

  /**
   * @brief calculate torque according to velocity
   * 
   * @param velocity 
   * @return double 
   */
  double calculate_torque(double velocity);

  /**
   * @brief convert rpm to speed in meters per second
   * 
   * @param rpm rotations per minute
   * @return speed in meters per second
   */
  double rpm_to_speed(double rpm);
  /**
   * @brief Contructor for the InspectionMission class
   */
  InspectionMission();
};