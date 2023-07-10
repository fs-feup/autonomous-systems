#include "planning/planning.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "utils/files.hpp"

using std::placeholders::_1;

Planning::Planning() : Node("planning"), initialOrientation_(-1) {
  this->vl_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
      "vehicle_localization", 10, std::bind(&Planning::vehicle_localisation_callback, this, _1));

  this->track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, std::bind(&Planning::track_map_callback, this, _1));

  this->local_pub_ =
      this->create_publisher<custom_interfaces::msg::PointArray>("planning_local", 10);
  this->global_pub_ =
      this->create_publisher<custom_interfaces::msg::PointArray>("planning_global", 10);

  this->adapter = new Adapter("eufs", this);
}

void Planning::vehicle_localisation_callback(const custom_interfaces::msg::Pose msg) {
  RCLCPP_INFO(this->get_logger(), "[localisation] (%f, \t%f, \t%fdeg)", msg.position.x,
              msg.position.y, msg.theta);
  if (initialOrientation_ == -1) {
    initialOrientation_ = msg.theta;
    local_path_planner->setOrientation(msg.theta);
    RCLCPP_INFO(this->get_logger(), "Orientation set to %f degrees.", initialOrientation_);
  }
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray msg) {
  Track* track = new Track();
  auto cone_array = msg.cone_array;

  for (auto& cone : cone_array) {
    track->addCone(cone.position.x, cone.position.y, cone.color);
    RCLCPP_INFO(this->get_logger(), "[received] (%f, \t%f)\t%s", cone.position.x, cone.position.y,
                cone.color.c_str());
  }

  std::vector<Position*> path = local_path_planner->processNewArray(track);
  publish_track_points(path);
  delete (track);

  RCLCPP_INFO(this->get_logger(), "--------------------------------------");
}

/**
 * Publisher point by point
 */
void Planning::publish_track_points(std::vector<Position*> path) const {
  auto message = custom_interfaces::msg::PointArray();
  for (auto const& element : path) {
    auto point = custom_interfaces::msg::Point2d();
    point.x = element->getX();
    point.y = element->getY();
    message.points.push_back(point);
    RCLCPP_INFO(this->get_logger(), "[published] (%f, \t%f)", point.x, point.y);
  }
  local_pub_->publish(message);
}

void Planning::set_mission(Mission mission) { this->mission = mission; }