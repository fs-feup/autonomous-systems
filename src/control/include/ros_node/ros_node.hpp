#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "config/parameters.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/evaluator_control_data.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "pid/pid.hpp"
#include "pure_pursuit/point_solver.hpp"
#include "pure_pursuit/pure_pursuit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

/**
 * @class Control
 * @brief Class responsible for the control of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and ideal path topics, and publishing torque (or other output to the actuators).
 */
class ControlNode : public rclcpp::Node {
public:
  bool go_signal_{false};
  float velocity_{0.0};
  double throttle_command_{0.0};
  double steering_command_{0.0};
  ControlParameters params_;

  explicit ControlNode(const ControlParameters &params);

  /**
   * @brief Publishes the steering angle to the car based on the path and pose using cache
   *
   */
  void publish_control(const custom_interfaces::msg::Pose &vehicle_state_msg);

private:
  // Evaluator Publisher
  rclcpp::Publisher<custom_interfaces::msg::EvaluatorControlData>::SharedPtr evaluator_data_pub_;

  // General Subscribers
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vehicle_pose_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr velocity_sub_;
  rclcpp::Subscription<custom_interfaces::msg::PathPointArray>::SharedPtr path_point_array_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_point_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  void control_timer_callback();

  std::vector<custom_interfaces::msg::PathPoint> pathpoint_array_{};
  PointSolver point_solver_; /**< Point Solver */
  PID long_controller_;
  PurePursuit lat_controller_; /**< Lateral Controller*/

  void publish_evaluator_data(double lookahead_velocity,
                              common_lib::structures::Position lookahead_point,
                              common_lib::structures::Position closest_point,
                              const custom_interfaces::msg::VehicleState &vehicle_state_msg,
                              double closest_point_velocity, double execution_time) const;

  virtual void publish_cmd(double acceleration, double steering) = 0;

  void publish_visualization_data(const common_lib::structures::Position &lookahead_point,
                                  const common_lib::structures::Position &closest_point) const;
};