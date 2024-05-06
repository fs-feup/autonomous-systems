#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "custom_interfaces/msg/control_command.h"
#include "custom_interfaces/msg/path_point_array.h"
#include "custom_interfaces/msg/pose.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Adapter;

/**
 * @class Control
 * @brief Class responsible for the control of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and ideal path topics, and publishing torque (or other output to the actuators).
 */
class Control : public rclcpp::Node {
 private:
  message_filters::Cache<custom_interfaces::msg::PathPointArray> path_cache;
  message_filters::Subscriber<custom_interfaces::msg::PathPointArray> path_point_array_sub;
  message_filters::Subscriber<custom_interfaces::msg::Pose> pose_sub;

  // TODO: change to correct message type
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr result;

  Adapter *adapter;
  std::string mode = "fsds";  // Temporary, change as desired. TODO(andre): Make not hardcoded

  /**
   * @brief Publishes the steering angle to the car based on the path and pose using timer
   * synchronizer
   */
  void publish_steering_angle_synchronized(
      const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg,
      const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg);

  /**
   * @brief Publishes the steering angle to the car based on the path and pose using cache
   * 
   */
  void publish_steering_angle_cached(const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg);

 public:
  /**
   * @brief Contructor for the Control class
   */
  Control();
};