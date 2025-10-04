#include "ros_node/ros_node.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/communication/marker.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "config/parameters.hpp"
#include "custom_interfaces/msg/evaluator_control_data.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This creates a subclass of Node and uses std::bind()
// to define which function gets executed at each time

using namespace common_lib::structures;
using namespace common_lib::communication;

bool received_vehicle_state = false;
bool received_path_point_array = false;

ControlNode::ControlNode(const ControlParameters& params)
    : Node("control"),
      params_(params),
      evaluator_data_pub_(create_publisher<custom_interfaces::msg::EvaluatorControlData>(
          "/control/evaluator_data", 10)),
      path_point_array_sub_(create_subscription<custom_interfaces::msg::PathPointArray>(
          params.use_simulated_planning_ ? "/path_planning/mock_path" : "/path_planning/path",
          rclcpp::QoS(10),
          [this](const custom_interfaces::msg::PathPointArray& msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received pathpoint array");
            pathpoint_array_ = msg.pathpoint_array;
            received_path_point_array = true;
          })),
      closest_point_pub_(create_publisher<visualization_msgs::msg::Marker>(
          "/control/visualization/closest_point", 10)),
      lookahead_point_pub_(create_publisher<visualization_msgs::msg::Marker>(
          "/control/visualization/lookahead_point", 10)),
      long_controller_(params_),
      lat_controller_(params_) {
  this->control_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(this->params_.command_time_interval_),
                              std::bind(&ControlNode::control_timer_callback, this));
  if (!params_.using_simulated_slam_) {
    vehicle_pose_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/state_estimation/vehicle_pose", 10,
        std::bind(&ControlNode::publish_control, this, std::placeholders::_1));
  }
  if (!params_.using_simulated_velocities_) {
    velocity_sub_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/state_estimation/velocities", 10,
        [this](const custom_interfaces::msg::Velocities::SharedPtr msg) {
          this->velocity_ =
              std::sqrt(msg->velocity_x * msg->velocity_x + msg->velocity_y * msg->velocity_y);
        });
  }
}

void ControlNode::control_timer_callback() {
  publish_cmd(this->throttle_command_, this->steering_command_);
}

// This function is called when a new pose is received
void ControlNode::publish_control(const custom_interfaces::msg::Pose& vehicle_state_msg) {
  if (!go_signal_) {
    RCLCPP_INFO(rclcpp::get_logger("control"), "Go Signal Not received");

    return;
  }
  this->vehicle_orientation_ = vehicle_state_msg.theta;

  if (received_path_point_array && !received_vehicle_state) {
    RCLCPP_DEBUG(rclcpp::get_logger("control"), "First Vehicle State Received");

    received_vehicle_state = true;
    custom_interfaces::msg::PathPoint initial;
    initial.x = vehicle_state_msg.x;
    initial.y = vehicle_state_msg.y;
    initial.v = this->velocity_;

    // Insert at the beginning of the pathpoint_array_ (true "push_front")
    pathpoint_array_.insert(pathpoint_array_.begin(), initial);
  }

  rclcpp::Time start = this->now();

  Position vehicle_cog = Position(vehicle_state_msg.x, vehicle_state_msg.y);
  Position rear_axis = rear_axis_position(vehicle_cog, vehicle_state_msg.theta,
      this->params_.car_parameters_.dist_cg_2_rear_axis);

  // find the closest point on the path
  // print pathpoint array size
  auto [closest_point, closest_point_id, closest_point_velocity] =
      get_closest_point(pathpoint_array_, rear_axis);
  if (closest_point_id == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("control"),
                 "PurePursuit: Failed to update closest point, size of pathpoint_array: %ld",
                 pathpoint_array_.size());
  }

  double ld = std::max(this->params_.pure_pursuit_lookahead_gain_ * this->velocity_,
                       this->params_.pure_pursuit_lookahead_minimum_);

  // update the Lookahead point
  auto [lookahead_point, lookahead_velocity, lookahead_error] =
      get_lookahead_point(pathpoint_array_, closest_point_id, ld, rear_axis, this->params_.pure_pursuit_first_last_max_dist_);

  if (lookahead_error) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update lookahead point");
  }

  // calculate longitudinal control: PI-D
  double torque = this->long_controller_.update(closest_point_velocity, this->velocity_);

  // calculate Lateral Control: Pure Pursuit
  double steering_angle = this->lat_controller_.pp_steering_control_law(
      rear_axis, vehicle_cog,
      lookahead_point, this->params_.car_parameters_.dist_cg_2_rear_axis);

  // check if steering is Nan
  if (std::isnan(steering_angle) || std::isnan(torque)) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "Steering Angle or Torque is NaN");
    return;
  }

  double execution_time = (this->now() - start).seconds() * 1000;

  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Current vehicle velocity: %f",
               this->velocity_);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Rear axis coords: %f, %f",
               rear_axis.x,
               rear_axis.y);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Closest Point: %f, %f, ID %d", closest_point.x,
               closest_point.y, closest_point_id);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Lookahead Point: %f, %f", lookahead_point.x,
               lookahead_point.y);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Lookahead Velocity: %f", lookahead_velocity);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Torque: %f, Steering Angle: %f", torque,
               steering_angle);

  publish_evaluator_data(rear_axis, lookahead_velocity, lookahead_point, closest_point,
                         closest_point_velocity, execution_time);
  publish_visualization_data(lookahead_point, closest_point);

  this->throttle_command_ = torque;
  this->steering_command_ = steering_angle;
}

void ControlNode::publish_evaluator_data(Position rear_axis,
    double lookahead_velocity, Position lookahead_point, Position closest_point,
    double closest_point_velocity,
    double execution_time) const {
  // Direct conversion of Pose to VehicleState, still needs a better solution, but works for now to
  // have control evalution data
  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.header = std_msgs::msg::Header();
  vehicle_state.position.x = rear_axis.x;
  vehicle_state.position.y = rear_axis.y;
  vehicle_state.theta = this->vehicle_orientation_;
  vehicle_state.linear_velocity = this->velocity_;
  vehicle_state.angular_velocity = 0.0;
  custom_interfaces::msg::EvaluatorControlData evaluator_data;
  evaluator_data.header = std_msgs::msg::Header();
  evaluator_data.header.stamp = this->now();
  evaluator_data.vehicle_state = vehicle_state;
  evaluator_data.vehicle_state.position.x = rear_axis.x;
  evaluator_data.vehicle_state.position.y = rear_axis.y;
  evaluator_data.lookahead_point.x = lookahead_point.x;
  evaluator_data.lookahead_point.y = lookahead_point.y;
  evaluator_data.closest_point.x = closest_point.x;
  evaluator_data.closest_point.y = closest_point.y;
  evaluator_data.lookahead_velocity = lookahead_velocity;
  evaluator_data.closest_point_velocity = closest_point_velocity;
  evaluator_data.execution_time = execution_time;
  this->evaluator_data_pub_->publish(evaluator_data);
}

void ControlNode::publish_visualization_data(const Position& lookahead_point,
                                             const Position& closest_point) const {
  auto lookahead_msg = common_lib::communication::marker_from_position(
      lookahead_point, "control", 0, "green", 0.5, this->params_.map_frame_id_);
  auto closest_msg = common_lib::communication::marker_from_position(
      closest_point, "control", 1, "red", 0.5, this->params_.map_frame_id_);

  this->closest_point_pub_->publish(closest_msg);
  this->lookahead_point_pub_->publish(lookahead_msg);
}
