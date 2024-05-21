#include "common_lib/communication/marker.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

// namespace common_lib::communication {
//
// visualization_msgs::msg::MarkerArray marker_array_from_cone_array(
//     std::vector<common_lib::structures::Cone> cone_array, std::string name_space,
//     std::string frame_id, std::string color, std::string shape, float scale, int action) {
//   visualization_msgs::msg::MarkerArray marker_array;
//   std::array<float, 4> color_array = marker_color_map.at(color);
//
//   for (size_t i = 0; i < cone_array.size(); ++i) {
//     visualization_msgs::msg::Marker marker;
//
//     marker.header.frame_id = frame_id;
//     marker.header.stamp = rclcpp::Clock().now();
//     marker.ns = name_space;
//     marker.id = i;
//     marker.type = marker_shape_map.at(shape);
//     marker.action = action;
//
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
//
//     marker.pose.position.x = cone_array[i].position.x;
//     marker.pose.position.y = cone_array[i].position.y;
//     marker.pose.position.z = 0;
//
//     marker.scale.x = scale;
//     marker.scale.y = scale;
//     marker.scale.z = scale;
//
//     marker.color.r = color_array[0];
//     marker.color.g = color_array[1];
//     marker.color.b = color_array[2];
//     marker.color.a = color_array[3];
//
//     marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(5));
//
//     marker_array.markers.push_back(marker);
//   }
//
//   return marker_array;
// }
//
// visualization_msgs::msg::MarkerArray marker_array_from_path_point_array(
//     std::vector<common_lib::structures::PathPoint> path_point_array, std::string name_space,
//     std::string frame_id, std::string color, std::string shape, float scale, int action) {
//   visualization_msgs::msg::MarkerArray marker_array;
//   std::array<float, 4> color_array = marker_color_map.at(color);
//
//   for (size_t i = 0; i < path_point_array.size(); ++i) {
//     visualization_msgs::msg::Marker marker;
//
//     marker.header.frame_id = frame_id;
//     marker.header.stamp = rclcpp::Clock().now();
//     marker.ns = name_space;
//     marker.id = i;
//     marker.type = marker_shape_map.at(shape);
//     marker.action = action;
//
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
//
//     marker.pose.position.x = path_point_array[i].position.x;
//     marker.pose.position.y = path_point_array[i].position.y;
//     marker.pose.position.z = 0;
//
//     marker.scale.x = scale;
//     marker.scale.y = scale;
//     marker.scale.z = scale;
//
//     marker.color.r = color_array[0];
//     marker.color.g = color_array[1];
//     marker.color.b = color_array[2];
//     marker.color.a = color_array[3];
//
//     marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(5));
//
//     marker_array.markers.push_back(marker);
//   }
//
//   return marker_array;
// }
//
//
// }  // namespace common_lib::communication