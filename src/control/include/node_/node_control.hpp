#ifndef NODE_CONTROL_HPP_
#define NODE_CONTROL_HPP_

#include "adapter_control/adapter.hpp"
//#include "adapter_control/map.hpp"
//#include "common_lib/structures/structures.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "pid/pid.hpp"
#include "point_solver/psolver.hpp"
#include "pure_pursuit/pp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//class Adapter;

constexpr double K = 1.0;        /**< PP_gain */
constexpr double LD_MARGIN = 0.1; /**< Lookahead distance margin */

/**
 * @class Control
 * @brief Class responsible for the control of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and ideal path topics, and publishing torque (or other output to the actuators).
 */
class Control : public rclcpp::Node {
 public:
  PointSolver point_solver;   /**< Point Solver */
  PID long_controller;        /**< Longitudinal Controller */
  PurePursuit lat_controller; /**< Lateral Controller*/

  // Need to change this so it is changed in the launch file
  double k_ = K;
  double ld_margin_ = LD_MARGIN;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_lat_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_long_pub;
  rclcpp::Publisher<custom_interfaces::msg::PathPoint>::SharedPtr lookahead_point_pub;
  rclcpp::Publisher<custom_interfaces::msg::PathPoint>::SharedPtr closest_point_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_velcoity;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr path_subscription;
  double velocity;

  Adapter *adapter;
  std::string mode = "fsds";  // Temporary, change as desired. TODO(andre): Make not hardcoded

  /**
   * @brief Function to publish the desired output (provisionally torque)
   * when a new map is recieved
   */
  void publish_torque(custom_interfaces::msg::ConeArray path);

  /**
   * @brief Function to hold the value of the velocity when
   * new velocity data is recieved
   */
  void velocity_estimation_callback(std_msgs::msg::String velocity);

  /**
   * @brief Orchestrator callback
   *
   * @param path_msg
   * @param pose_msg
   */
  void orchestrator_callback(
      const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg,
      const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg);

  /*
   * @brief Publish lookahead point
   */
  void publish_lookahead_point(Point lookahead_point, double lookahead_velocity);

  /*
   * @brief Publish closest point
   */
  void publish_closest_point(Point closest_point);

  /*
   * @brief Publish Torque command
   */
  void publish_torque(float torque);

  /*
   * @brief Publish Steering command
   */
  void publish_steering(double steering);

  /**
   * @brief Update lookahead distance
   */
  double update_lookahead_distance(double k, double velocity);

  /**
   * @brief Contructor for the Control class
   */
  Control();
};
#endif//NODE_CONTROL_HPP_