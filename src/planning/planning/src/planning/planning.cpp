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

  // test only
  // std::cout << "Testing planning from file.\n";
  // Track* track = read_track_file("hairpins.txt");
  // std::vector<Position*> fullPath = local_path_planner->processNewArray(track);
  // write_path_file("finalPath.txt", fullPath);
  // std::cout << "Writing test planning to file with size " << fullPath.size() << "\n";
}

void Planning::vehicle_localisation_callback(const custom_interfaces::msg::Pose msg) {
  RCLCPP_INFO(this->get_logger(), "Vehicle Localisation: (%f, %f, %fdeg)", msg.position.x,
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

  try {
    std::vector<Position*> path = local_path_planner->processNewArray(track);
    RCLCPP_INFO(this->get_logger(), "Processed!");
    publish_track_points(path);
    RCLCPP_INFO(this->get_logger(), "Published!");
    delete (track);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    throw std::runtime_error("Planning runtime error!");
  }

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
  }
  local_pub_->publish(message);

  RCLCPP_INFO(this->get_logger(), "[published] %ld points", message.points.size());
}

void Planning::set_mission(Mission mission) { this->mission = mission; }