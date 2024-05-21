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
      pose_sub_(this, "/state_estimation/vehicle_state"),
      path_point_array_sub_(this, declare_parameter("mocker_node", true)
                                      ? "/planning/mock/ground_truth"
                                      : "/path_planning/path"),
      path_cache_(path_point_array_sub_, 10),
      adapter_(adapter_map.at(declare_parameter("adapter", "vehicle"))(this)) {
        
  pose_sub_.registerCallback(&Control::publish_control, this);
}

// This function is called when a new pose is received
void Control::publish_control(
    const custom_interfaces::msg::VehicleState::ConstSharedPtr &pose_msg) {
  if (!go_signal) return;
  auto path_point_array = path_cache_.getElemBeforeTime(pose_msg->header.stamp);
  auto control_msg = custom_interfaces::msg::ControlCommand();

  // BARROS FILL IN USING pose_msg and path_point_array, then fill in the control_msg

  adapter_->publish_cmd(control_msg.throttle, control_msg.steering);
  // Adapter to communicate with the car
  //this->adapter = adapter_map[mode](this);
}

void Control::orchestrator_callback(
    const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg,
    const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg) {
  
  // update vehicle pose
  this->point_solver_.update_vehicle_pose(pose_msg);

  // calculate lookahead distance
  double ld = this->k_* this->point_solver_.vehicle_pose_.velocity_;

  // find the closest point on the path
  auto [closest_point, closest_point_id] = this->point_solver_.update_closest_point(path_msg, this->point_solver_.vehicle_pose_.rear_axis_);

  // Publish closest point data
  publish_closest_point(closest_point);

  // update the Lookahead point
  auto [lookahead_point, lookahead_velocity, lookahead_error] =
      this->point_solver_.update_lookahead_point(path_msg, closest_point, closest_point_id,
                                                ld, this->ld_margin_);
  if (lookahead_error) {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "PurePursuit: Failed to update lookahed point");
  }

  // Publish lookahead point data
  publish_lookahead_point(lookahead_point, lookahead_velocity);

  // calculate longitudinal control: PI-D
  double torque =
      this->long_controller_.update(lookahead_velocity, this->point_solver_.vehicle_pose_.velocity_);

  // publish torque command
  publish_torque(torque);

  // calculate Lateral Control: Pure Pursuit
  double steering_angle = this->lat_controller_.pp_steering_control_law(
      this->point_solver_.vehicle_pose_.rear_axis_, this->point_solver_.vehicle_pose_.cg_,
      lookahead_point, this->point_solver_.dist_cg_2_rear_axis_, this->lat_controller_.wheel_base_,
      this->lat_controller_.max_steering_angle_, this->lat_controller_.min_steering_angle_);

  // publish steering command
  publish_steering(steering_angle);
  return;
}

void Control::publish_lookahead_point(Point lookahead_point, double lookahead_velocity)const {
  custom_interfaces::msg::PathPoint lookahead_point_msg;
  lookahead_point_msg.x = lookahead_point.x_;
  lookahead_point_msg.y = lookahead_point.y_;
  lookahead_point_msg.v = lookahead_velocity;
  this->lookahead_point_pub_->publish(lookahead_point_msg);
}

void Control::publish_closest_point(Point closest_point)const {
  custom_interfaces::msg::PathPoint closest_point_msg;
  closest_point_msg.x = closest_point.x_;
  closest_point_msg.y = closest_point.y_;
  closest_point_msg.v = 0;
  this->closest_point_pub_->publish(closest_point_msg);
}

void Control::publish_torque(double torque)const {
  std_msgs::msg::String torque_cmd;
  torque_cmd.data = std::to_string(torque);
  this->result_long_pub_->publish(torque_cmd);
}

void Control::publish_steering(double steering)const {
  std_msgs::msg::String steering_cmd;
  steering_cmd.data = std::to_string(steering);
  this->result_lat_pub_->publish(steering_cmd);
}