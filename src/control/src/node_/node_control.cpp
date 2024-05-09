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

// This function is called when a new pose is received
void Control::publish_control(
    const custom_interfaces::msg::VehicleState::ConstSharedPtr &pose_msg) {
  if (!go_signal) return;
  auto path_point_array = path_cache.getElemBeforeTime(pose_msg->header.stamp);
  auto control_msg = custom_interfaces::msg::ControlCommand();

  // BARROS FILL IN USING pose_msg and path_point_array, then fill in the control_msg

  result->publish(control_msg);
}

Control::Control()
    : Node("control"),
      pose_sub(this, "/state_estimation/vehicle_state"),
      path_point_array_sub(this, "/path_planning/path"),
      path_cache(path_point_array_sub, 10),
      go_sub(this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          [this](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
            go_signal = msg->go_signal;
          })) {
  pose_sub.registerCallback(&Control::publish_control, this);

  // creates publisher that should yield torque/acceleration/...
  result = this->create_publisher<custom_interfaces::msg::ControlCommand>("/as_msgs/controls", 10);

  // Adapter to communicate with the car
  this->adapter = adapter_map[mode](this);
}