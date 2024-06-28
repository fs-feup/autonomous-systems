#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/evaluator_control_data.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "pid/pid.hpp"
#include "point_solver/psolver.hpp"
#include "pure_pursuit/pp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

struct ControlParameters {
  bool using_simulated_se_;
  bool mocker_node_;
  double lookahead_gain_;
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
  bool using_simulated_se_{false};
  bool go_signal_{false};

  explicit Control(const ControlParameters &params);

  /**
   * @brief Publishes the steering angle to the car based on the path and pose using cache
   *
   */
  void publish_control(const custom_interfaces::msg::VehicleState &vehicle_state_msg);

private:
  bool mocker_node_{false};
  // std::string adapter_;

  // Evaluator Publisher
  rclcpp::Publisher<custom_interfaces::msg::EvaluatorControlData>::SharedPtr evaluator_data_pub_;

  // General Subscribers
  rclcpp::Subscription<custom_interfaces::msg::VehicleState>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<custom_interfaces::msg::PathPointArray>::SharedPtr path_point_array_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_point_pub_;

  std::vector<custom_interfaces::msg::PathPoint> pathpoint_array_{};
  PointSolver point_solver_; /**< Point Solver */
  PID long_controller_{0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, 0.7};
  PurePursuit lat_controller_; /**< Lateral Controller*/

  void publish_evaluator_data(double lookahead_velocity,
                              common_lib::structures::Position lookahead_point,
                              common_lib::structures::Position closest_point,
                              custom_interfaces::msg::VehicleState ,
                              double closest_point_velocity) const;

  virtual void publish_cmd(double acceleration, double steering) = 0;

  void publish_visualization_data(const common_lib::structures::Position &lookahead_point,
                                  const common_lib::structures::Position &closest_point) const;
};