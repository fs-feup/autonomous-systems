#pragma once

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "pid/pid.hpp"
#include "point_solver/psolver.hpp"
#include "pure_pursuit/pp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr double K = 1.0;         /**< PP_gain */
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
  double k_;
  double ld_margin_;
  bool using_simulated_se_{false};
  bool go_signal_{false};
  bool mocker_node_{false};

  PointSolver point_solver_;   /**< Point Solver */
  PID long_controller_;        /**< Longitudinal Controller */
  PurePursuit lat_controller_; /**< Lateral Controller*/

  std::shared_ptr<Adapter> adapter_;

  // Evaluator Publishers
  rclcpp::Publisher<custom_interfaces::msg::PathPoint>::SharedPtr lookahead_point_pub_;
  rclcpp::Publisher<custom_interfaces::msg::PathPoint>::SharedPtr closest_point_pub_;

  // General Subscribers
  rclcpp::Subscription<custom_interfaces::msg::PathPointArray>::SharedPtr path_point_array_sub_;
  rclcpp::Subscription<custom_interfaces::msg::VehicleState>::SharedPtr vehicle_state_sub_;

  std::vector<custom_interfaces::msg::PathPoint> pathpoint_array_{};


  /**
   * @brief Publishes the steering angle to the car based on the path and pose using cache
   *
   */
  void publish_control(const custom_interfaces::msg::VehicleState &vehicle_state_msg);

 private:
  /*
   * @brief Publish lookahead point
   */
  void publish_lookahead_point(Point lookahead_point, double lookahead_velocity) const;

  /*
   * @brief Publish closest point
   */
  void publish_closest_point(Point closest_point) const;

  /**
   * @brief Update lookahead distance
   */
  double update_lookahead_distance(double k, double velocity) const;

  /**
   * @brief Contructor for the Control class
   */
 public:
  Control();
};