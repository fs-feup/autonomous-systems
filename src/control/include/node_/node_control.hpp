#pragma once

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/evaluator_control_data.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "pid/pid.hpp"
#include "point_solver/psolver.hpp"
#include "pure_pursuit/pp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class Control
 * @brief Class responsible for the control of the car
 *
 * This class inherits from rclcpp::Node, subscribing to current velocity
 * and ideal path topics, and publishing torque (or other output to the actuators).
 */
class Control : public rclcpp::Node {
public:
  double k_;
  double ld_margin_;
  bool using_simulated_se_{false};
  bool go_signal_{false};
  bool mocker_node_{false};

  PointSolver point_solver_; /**< Point Solver */
  PID long_controller_{0.4, 0.3, 0.09, 0.5, 0.01, -1, 1, 0.7};
  PurePursuit lat_controller_; /**< Lateral Controller*/

  std::shared_ptr<Adapter> adapter_;

  // Evaluator Publishers
  rclcpp::Publisher<custom_interfaces::msg::EvaluatorControlData>::SharedPtr evaluator_data_pub_;

  // General Subscribers
  rclcpp::Subscription<custom_interfaces::msg::PathPointArray>::SharedPtr path_point_array_sub_;
  rclcpp::Subscription<custom_interfaces::msg::VehicleState>::SharedPtr vehicle_state_sub_;

  std::vector<custom_interfaces::msg::PathPoint> pathpoint_array_{};

  /**
   * @brief Contructor for the Control class
   */
  Control();

  /**
   * @brief Publishes the steering angle to the car based on the path and pose using cache
   *
   */
  void publish_control(const custom_interfaces::msg::VehicleState &vehicle_state_msg);

private:
  void publish_evaluator_data(double lookahead_velocity, Point lookahead_point, Point closest_point,
                              custom_interfaces::msg::VehicleState vehicle_state_msg) const;
};