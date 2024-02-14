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

Planning::Planning() : Node("planning") {
  // LocMap Subscriber
  this->track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, std::bind(&Planning::track_map_callback, this, _1));

  // Control Publishers
  this->local_pub_ =
      this->create_publisher<custom_interfaces::msg::PointArray>("planning_local", 10);
  this->global_pub_ =
      this->create_publisher<custom_interfaces::msg::PointArray>("planning_global", 10);

  // Publishes path from file in Skidpad & Acceleration events
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&Planning::publish_predicitive_track_points, this));

  // Adapter to communicate with the car
  this->adapter = new Adapter("eufs", this);
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray msg) {
  if (this->is_predicitve_mission()) {
    return;
  }

  Track *track = new Track();

  auto cone_array = msg.cone_array;

  for (auto &cone : cone_array) {
    RCLCPP_DEBUG(this->get_logger(), "[received] (%f, %f)\t%s", cone.position.x, cone.position.y,
                 cone.color.c_str());
    track->addCone(cone.position.x, cone.position.y, cone.color);
  }

  track->validateCones();  // Delete cone outliers

  std::vector<PathPoint *> path = local_path_planner->processNewArray(track);  // Calculate Path
  publish_track_points(path);
  delete (track);

  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
}

/**
 * Publisher point by point
 */
void Planning::publish_track_points(std::vector<PathPoint *> path) const {
  auto message = custom_interfaces::msg::PointArray();
  for (auto const &element : path) {
    auto point = custom_interfaces::msg::Point2d();
    point.x = element->getX();
    point.y = element->getY();
    message.points.push_back(point);
    RCLCPP_DEBUG(this->get_logger(), "[published] (%f, %f)", point.x, point.y);
  }
  local_pub_->publish(message);
}

void Planning::publish_predicitive_track_points() {
  // RCLCPP_INFO(this->get_logger(), "[mission] (%d)", this->mission);
  if (!this->is_predicitve_mission()) {
    return;
  }
  std::vector<PathPoint *> path = read_path_file(this->predictive_paths[this->mission]);
  this->publish_track_points(path);
}

void Planning::set_mission(Mission mission) { this->mission = mission; }

bool Planning::is_predicitve_mission() const {
  return this->mission == Mission::skidpad || this->mission == Mission::acceleration;
}