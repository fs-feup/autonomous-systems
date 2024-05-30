#include "node_/node_control.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "adapter_control/map.hpp"
#include "custom_interfaces/msg/evaluator_control_data.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This creates a subclass of Node and uses std::bind()
// to define which function gets executed at each time

Control::Control()
    : Node("node_control"),
      using_simulated_se_(declare_parameter("use_simulated_se", false)),
      adapter_(adapter_map.at(declare_parameter("adapter", "vehicle"))(this)),
      mocker_node_(declare_parameter("mocker_node", true)),
      evaluator_data_pub_(create_publisher<custom_interfaces::msg::EvaluatorControlData>(
          "/control/evaluator_data", 10)),
      path_point_array_sub_(create_subscription<custom_interfaces::msg::PathPointArray>(
          mocker_node_ ? "/path_planning/mock_path" : "/path_planning/path", rclcpp::QoS(10),
          [this](const custom_interfaces::msg::PathPointArray& msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received pathpoint array");
            pathpoint_array_ = msg.pathpoint_array;
          })),
      point_solver_(declare_parameter("lookahead_gain", 0.5),
                    declare_parameter("lookahead_margin", 0.1)) {
  if (!using_simulated_se_ || get_parameter("adapter").as_string() == "vehicle") {
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
  auto [closest_point, closest_point_id] = this->point_solver_.update_closest_point(
      pathpoint_array_, this->point_solver_.vehicle_pose_.rear_axis_);
  if (closest_point_id == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update closest point");
    return;
  }

  // update the Lookahead point
  auto [lookahead_point, lookahead_velocity, lookahead_error] =
      this->point_solver_.update_lookahead_point(pathpoint_array_, closest_point, closest_point_id);
  if (lookahead_error) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update lookahed point");
    return;
  }

  // calculate longitudinal control: PI-D
  double torque = this->long_controller_.update(lookahead_velocity,
                                                this->point_solver_.vehicle_pose_.velocity_);

  // calculate Lateral Control: Pure Pursuit
  double steering_angle = this->lat_controller_.pp_steering_control_law(
      this->point_solver_.vehicle_pose_.rear_axis_, this->point_solver_.vehicle_pose_.cg_,
      lookahead_point, this->point_solver_.dist_cg_2_rear_axis_);

  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Current vehicle velocity: %f",
               this->point_solver_.vehicle_pose_.velocity_);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Rear axis coords: %f, %f",
               this->point_solver_.vehicle_pose_.rear_axis_.x_,
               this->point_solver_.vehicle_pose_.rear_axis_.y_);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Closest Point: %f, %f", closest_point.x_,
               closest_point.y_);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Lookahead Point: %f, %f", lookahead_point.x_,
               lookahead_point.y_);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Torque: %f, Steering Angle: %f", torque,
               steering_angle);

  publish_evaluator_data(lookahead_velocity, lookahead_point, closest_point, vehicle_state_msg);
  adapter_->publish_cmd(torque, steering_angle);
  // Adapter to communicate with the car
}

void Control::publish_evaluator_data(double lookahead_velocity, Point lookahead_point,
                                     Point closest_point,
                                     custom_interfaces::msg::VehicleState vehicle_state_msg) const {
  custom_interfaces::msg::EvaluatorControlData evaluator_data;
  evaluator_data.header = std_msgs::msg::Header();
  evaluator_data.header.stamp = this->now();
  evaluator_data.vehicle_state = vehicle_state_msg;
  evaluator_data.vehicle_state.position.x = this->point_solver_.vehicle_pose_.rear_axis_.x_;
  evaluator_data.vehicle_state.position.y = this->point_solver_.vehicle_pose_.rear_axis_.y_;
  evaluator_data.lookahead_point.x = lookahead_point.x_;
  evaluator_data.lookahead_point.y = lookahead_point.y_;
  evaluator_data.closest_point.x = closest_point.x_;
  evaluator_data.closest_point.y = closest_point.y_;
  evaluator_data.lookahead_velocity = lookahead_velocity;
  this->evaluator_data_pub_->publish(evaluator_data);
}