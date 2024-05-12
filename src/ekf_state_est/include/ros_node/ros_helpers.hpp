#pragma once

#include "common_lib/structures/cone.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/**
 * @brief Converts a vector of cones to a marker array
 * TODO: improve this function
 *
 * @param cone_array vector of cones
 * @return visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray marker_array_from_cone_array(
    std::vector<common_lib::structures::Cone> cone_array) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < cone_array.size(); ++i) {
    visualization_msgs::msg::Marker marker;

    // TODO: dynamic frame_id
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "cones";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::MODIFY;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.pose.position.x = cone_array[i].position.x;
    marker.pose.position.y = cone_array[i].position.y;
    marker.pose.position.z = 0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}