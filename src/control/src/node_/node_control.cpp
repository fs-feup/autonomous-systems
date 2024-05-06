#include "node_/node_control.hpp"

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adapter_control/adapter.hpp"
#include "adapter_control/map.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This creates a subclass of Node and uses std::bind()
// to define which function gets executed at each time

// This function is called when a new path and pose are received
void Control::publish_steering_angle_synchronized(
    const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg,
    const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg) {
  // BARROS FILL IN
}

// This function is called when a new pose is received
void Control::publish_steering_angle_cached(
    const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg) {
  // TODO: Fix missing header from pose_msg
  auto path_point_array = path_cache.getElemBeforeTime(pose_msg->header.stamp);
  // BARROS FILL IN
}

// TODO: Change to correct topics
Control::Control()
    : Node("control"),
      path_point_array_sub(this, "path_topic"),
      pose_sub(this, "pose_topic"),
      path_cache(path_point_array_sub, 10) {
  // Approach with Time Synchronized
  message_filters::TimeSynchronizer<custom_interfaces::msg::PathPointArray,
                                    custom_interfaces::msg::Pose>
      sync(path_point_array_sub, pose_sub, 10);
  sync.registerCallback(&publish_steering_angle_synchronized, this);

  // Approach with Cache
  pose_sub.registerCallback(&publish_steering_angle_cached, this);

  // creates publisher that should yield torque/acceleration/...
  // TODO: change to correct message type
  result = this->create_publisher<std_msgs::msg::String>("torque_topic", 10);

  // Adapter to communicate with the car
  this->adapter = adapter_map[mode](this);
}