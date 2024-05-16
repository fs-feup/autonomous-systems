#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
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
  message_filters::Subscriber<custom_interfaces::msg::VehicleState> pose_sub_;
  message_filters::Subscriber<custom_interfaces::msg::PathPointArray> path_point_array_sub_;
  message_filters::Cache<custom_interfaces::msg::PathPointArray> path_cache_;

  Adapter *adapter_;
  std::string adapter_mode;

  /**
   * @brief Publishes the steering angle to the car based on the path and pose using cache
   *
   */
  void publish_control(const custom_interfaces::msg::VehicleState::ConstSharedPtr &pose_msg);

 public:
  bool go_signal{false};
  /**
   * @brief Contructor for the Control class
   */
  Control();
};