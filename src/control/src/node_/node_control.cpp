#include "node_/node_control.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/communication/marker.hpp"
#include "custom_interfaces/msg/evaluator_control_data.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This creates a subclass of Node and uses std::bind()
// to define which function gets executed at each time

using namespace common_lib::structures;

using namespace common_lib::communication;

Control::Control(const ControlParameters& params)
    : Node("control"),
      using_simulated_se_(params.using_simulated_se_),
      mocker_node_(params.mocker_node_),
      evaluator_data_pub_(create_publisher<custom_interfaces::msg::EvaluatorControlData>(
          "/control/evaluator_data", 10)),
      path_point_array_sub_(create_subscription<custom_interfaces::msg::PathPointArray>(
          mocker_node_ ? "/path_planning/mock_path" : "/path_planning/path", rclcpp::QoS(10),
          [this](const custom_interfaces::msg::PathPointArray& msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received pathpoint array");
            pathpoint_array_ = msg.pathpoint_array;
          })),
      closest_point_pub_(create_publisher<visualization_msgs::msg::Marker>(
          "/control/visualization/closest_point", 10)),
      lookahead_point_pub_(create_publisher<visualization_msgs::msg::Marker>(
          "control/visualization/lookahead_point", 10)),
      point_solver_(params.lookahead_gain_) {
  if (!using_simulated_se_) {
    vehicle_state_sub_ = this->create_subscription<custom_interfaces::msg::VehicleState>(
        "/state_estimation/vehicle_state", 10,
        std::bind(&Control::publish_control, this, std::placeholders::_1));
  }
}

// This function is called when a new pose is received
void Control::publish_control(const custom_interfaces::msg::VehicleState& vehicle_state_msg) {
  if (!go_signal_) return;
  // update vehicle pose
  this->point_solver_.update_vehicle_pose(vehicle_state_msg);

  // find the closest point on the path
  // print pathpoint array size
  auto [closest_point, closest_point_id] =
      this->point_solver_.update_closest_point(pathpoint_array_);
  if (closest_point_id == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update closest point");
    return;
  }

  // update the Lookahead point
  auto [lookahead_point, lookahead_velocity, lookahead_error] =
      this->point_solver_.update_lookahead_point(pathpoint_array_, closest_point_id);
  if (lookahead_error) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update lookahed point");
    return;
  }

  // calculate longitudinal control: PI-D
  double torque = this->long_controller_.update(lookahead_velocity,
                                                this->point_solver_.vehicle_pose_.velocity_);

  // calculate Lateral Control: Pure Pursuit
  double steering_angle = this->lat_controller_.pp_steering_control_law(
      this->point_solver_.vehicle_pose_.rear_axis_, this->point_solver_.vehicle_pose_.position,
      lookahead_point, this->point_solver_.dist_cg_2_rear_axis_);

  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Current vehicle velocity: %f",
               this->point_solver_.vehicle_pose_.velocity_);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Rear axis coords: %f, %f",
               this->point_solver_.vehicle_pose_.rear_axis_.x,
               this->point_solver_.vehicle_pose_.rear_axis_.y);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Closest Point: %f, %f", closest_point.x,
               closest_point.y);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Lookahead Point: %f, %f", lookahead_point.x,
               lookahead_point.y);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Lookahead Velocity: %f", lookahead_velocity);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Torque: %f, Steering Angle: %f", torque,
               steering_angle);

  publish_evaluator_data(lookahead_velocity, lookahead_point, closest_point, vehicle_state_msg);
  publish_visualization_data(lookahead_point, closest_point);
  publish_cmd(torque, steering_angle);
  // Adapter to communicate with the car
}

void Control::publish_evaluator_data(double lookahead_velocity, Position lookahead_point,
                                     Position closest_point,
                                     custom_interfaces::msg::VehicleState vehicle_state_msg) const {
  custom_interfaces::msg::EvaluatorControlData evaluator_data;
  evaluator_data.header = std_msgs::msg::Header();
  evaluator_data.header.stamp = this->now();
  evaluator_data.vehicle_state = vehicle_state_msg;
  evaluator_data.vehicle_state.position.x = this->point_solver_.vehicle_pose_.rear_axis_.x;
  evaluator_data.vehicle_state.position.y = this->point_solver_.vehicle_pose_.rear_axis_.y;
  evaluator_data.lookahead_point.x = lookahead_point.x;
  evaluator_data.lookahead_point.y = lookahead_point.y;
  evaluator_data.closest_point.x = closest_point.x;
  evaluator_data.closest_point.y = closest_point.y;
  evaluator_data.lookahead_velocity = lookahead_velocity;
  this->evaluator_data_pub_->publish(evaluator_data);
}

void Control::publish_visualization_data(const Position& lookahead_point,
                                         const Position& closest_point) const {
  auto lookahead_msg =
      common_lib::communication::marker_from_position(lookahead_point, "control", 0, "green");
  auto closest_msg =
      common_lib::communication::marker_from_position(closest_point, "control", 1, "red");

  this->closest_point_pub_->publish(lookahead_msg);
  this->lookahead_point_pub_->publish(closest_msg);
}