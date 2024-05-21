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
}