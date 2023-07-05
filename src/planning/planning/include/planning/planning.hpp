#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PLANNING_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PLANNING_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "adapter/adapter.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "utils/files.hpp"

using std::placeholders::_1;

enum Mission { acceleration, skidpad, trackdrive, autocross };

class Planning : public rclcpp::Node {
  Mission mission = acceleration;
  LocalPathPlanner* local_path_planner = new LocalPathPlanner();
  Adapter* adapter;
  float initialOrientation_;

  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vl_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
  rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr local_pub_;
  rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr global_pub_;

  void vehicle_localisation_callback(const custom_interfaces::msg::Pose msg);

  void track_map_callback(const custom_interfaces::msg::ConeArray msg);

  void publish_track_points(std::vector<Position*> path) const;

 public:
  Planning();
  void set_mission(Mission mission);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PLANNING_HPP_