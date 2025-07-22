#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/evaluator_control_data.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "pid/pid.hpp"
#include "point_solver/psolver.hpp"
#include "pure_pursuit/pp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

struct ControlParameters {
  int16_t mission_;
  bool using_simulated_slam_;
  bool using_simulated_velocities_;
  bool use_simulated_planning_;
  double lookahead_gain_;
  double lookahead_minimum_;
  double pid_kp_;
  double pid_ki_;
  double pid_kd_;
  double pid_tau_;
  double pid_t_;
  double pid_lim_min_;
  double pid_lim_max_;
  double pid_anti_windup_;
  double lpf_alpha_;
  double lpf_initial_value_;
  std::string map_frame_id_;
  uint command_time_interval_;
  std::string test_mode_;
  double const_torque_value_;
  double ebs_torque_value_;
};

/**
 * @class Control
 * @brief Class responsible for the control of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and ideal path topics, and publishing torque (or other output to the actuators).
 */
class Control : public rclcpp::Node {
public:
  bool using_simulated_slam_{false};
  bool using_simulated_velocities_{false};
  bool go_signal_{false};
  float velocity_{0.0};
  double throttle_command_{0.0};
  double steering_command_{0.0};
  ControlParameters params_;

  explicit Control(const ControlParameters &params);

  /**
   * @brief Publishes the steering angle to the car based on the path and pose using cache
   *
   */
  void publish_control(const custom_interfaces::msg::Pose &vehicle_state_msg);

  static ControlParameters load_config(std::string &adapter);

private:
  bool use_simulated_planning_{false};
  std::string _map_frame_id_;

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