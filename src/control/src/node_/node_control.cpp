#include "node_/node_control.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "adapter_control/map.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This creates a subclass of Node and uses std::bind()
// to define which function gets executed at each time

Control::Control()
    : Node("node_control"),
      k_(declare_parameter("lookahead_gain", 0.5)),
      ld_margin_(declare_parameter("lookahead_margin", 0.1)),
      using_simulated_se_(declare_parameter("use_simulated_se", false)),
      mocker_node_(declare_parameter("mocker_node", true)),
      adapter_(adapter_map.at(declare_parameter("adapter", "vehicle"))(this)),
      lookahead_point_pub_(
          create_publisher<custom_interfaces::msg::PathPoint>("control/lookahead_point", 10)),
      closest_point_pub_(
          create_publisher<custom_interfaces::msg::PathPoint>("control/closest_point", 10)),
      path_point_array_sub_(create_subscription<custom_interfaces::msg::PathPointArray>(
          mocker_node_ ? "path_planning/mock_path" : "/path_planning/path",
          rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
          [this](const custom_interfaces::msg::PathPointArray& msg) {
            pathpoint_array_ = msg.pathpoint_array;
          })) {
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
  RCLCPP_INFO(rclcpp::get_logger("control"), "Current vehicle velocity: %f",
              this->point_solver_.vehicle_pose_.velocity_);
  RCLCPP_INFO(rclcpp::get_logger("control"), "Rear axis coords: %f, %f",
              this->point_solver_.vehicle_pose_.rear_axis_.x_,
              this->point_solver_.vehicle_pose_.rear_axis_.y_);

  // calculate lookahead distance
  double ld = this->k_ * std::max(this->point_solver_.vehicle_pose_.velocity_, 10.0);
  RCLCPP_INFO(rclcpp::get_logger("control"), "Current ld: %f", ld);

  // find the closest point on the path
  // print pathpoint array size
  auto [closest_point, closest_point_id] = this->point_solver_.update_closest_point(
      pathpoint_array_, this->point_solver_.vehicle_pose_.rear_axis_);
  if (closest_point_id == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update closest point");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("control"), "Closest Point: %f, %f", closest_point.x_,
              closest_point.y_);
  // Publish closest point data
  publish_closest_point(closest_point);

  // update the Lookahead point
  auto [lookahead_point, lookahead_velocity, lookahead_error] =
      this->point_solver_.update_lookahead_point(pathpoint_array_, closest_point, closest_point_id,
                                                 ld, this->ld_margin_);
  if (lookahead_error) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update lookahed point");
    return;
  }

  // Publish lookahead point data
  RCLCPP_INFO(rclcpp::get_logger("control"), "Lookahead Point: %f, %f", lookahead_point.x_,
              lookahead_point.y_);
  publish_lookahead_point(lookahead_point, lookahead_velocity);

  // calculate longitudinal control: PI-D
  double torque = this->long_controller_.update(lookahead_velocity,
                                                this->point_solver_.vehicle_pose_.velocity_);
  RCLCPP_INFO(rclcpp::get_logger("control"), "Torque from long control update is: %f", torque);

  // calculate Lateral Control: Pure Pursuit
  double steering_angle = this->lat_controller_.pp_steering_control_law(
      this->point_solver_.vehicle_pose_.rear_axis_, this->point_solver_.vehicle_pose_.cg_,
      lookahead_point, this->point_solver_.dist_cg_2_rear_axis_, this->lat_controller_.wheel_base_,
      this->lat_controller_.max_steering_angle_, this->lat_controller_.min_steering_angle_);

  RCLCPP_INFO(rclcpp::get_logger("control"), "Torque: %f, Steering Angle: %f", torque,
              steering_angle);
  adapter_->publish_cmd(torque, steering_angle);
  // Adapter to communicate with the car
  //
}

void Control::publish_lookahead_point(Point lookahead_point, double lookahead_velocity) const {
  custom_interfaces::msg::PathPoint lookahead_point_msg;
  lookahead_point_msg.x = lookahead_point.x_;
  lookahead_point_msg.y = lookahead_point.y_;
  lookahead_point_msg.v = lookahead_velocity;
  this->lookahead_point_pub_->publish(lookahead_point_msg);
}

void Control::publish_closest_point(Point closest_point) const {
  custom_interfaces::msg::PathPoint closest_point_msg;
  closest_point_msg.x = closest_point.x_;
  closest_point_msg.y = closest_point.y_;
  closest_point_msg.v = 0;
  this->closest_point_pub_->publish(closest_point_msg);
}
