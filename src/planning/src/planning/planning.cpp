#include "planning/planning.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "adapter_planning/eufs.hpp"
#include "adapter_planning/fsds.hpp"
#include "adapter_planning/map.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "planning/cone_coloring.hpp"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "planning/path_smoothing.hpp"
#include "utils/files.hpp"

using std::placeholders::_1;

Planning::Planning() : Node("planning") {
  angle_gain_ = declare_parameter<double>("angle_gain_", 8.7);
  distance_gain_ = declare_parameter<double>("distance_gain_", 15.0);
  ncones_gain_ = declare_parameter<double>("ncones_gain_", 8.7);
  angle_exponent_ = declare_parameter<double>("angle_exponent_", 0.7);
  distance_exponent_ = declare_parameter<double>("distance_exponent_", 1.7);
  cost_max_ = declare_parameter<double>("cost_max_", 40);
  outliers_spline_order_ = declare_parameter<int>("outliers_spline_order_", 3);
  outliers_spline_coeffs_ratio_ = declare_parameter<float>("outliers_spline_coeffs_ratio_", 3.0);
  outliers_spline_precision_ = declare_parameter<int>("outliers_spline_precision_", 1);
  smoothing_spline_order_ = declare_parameter<int>("smoothing_spline_order_", 3);
  smoothing_spline_coeffs_ratio_ = declare_parameter<float>("smoothing_spline_coeffs_ratio_", 3.0);
  smoothing_spline_precision_ = declare_parameter<int>("smoothing_spline_precision_", 10);
  mode = declare_parameter<std::string>("adapter", "fsds");

  // State Estimation map Subscriber
  this->track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "/state_estimation/map", 10, std::bind(&Planning::track_map_callback, this, _1));

  // Vehicle Localization Subscriber
  this->vl_sub_ = this->create_subscription<custom_interfaces::msg::VehicleState>(
      "/state_estimation/vehicle_state", 10,
      std::bind(&Planning::vehicle_localization_callback, this, _1));

  // Control Publishers
  this->local_pub_ =
      this->create_publisher<custom_interfaces::msg::PathPointArray>("local_planning", 10);

  // Publishes path from file in Skidpad & Acceleration events
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&Planning::publish_predicitive_track_points, this));

  // Adapter to communicate with the car
  this->adapter = adapter_map[mode](this);
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray &msg) {
  if (this->is_predicitve_mission()) {
    return;
  }

  auto track = new Track(this->outliers_spline_order_, this->outliers_spline_coeffs_ratio_,
                         this->outliers_spline_precision_);
  auto cone_coloring = std::make_shared<ConeColoring>(this->angle_gain_, this->distance_gain_,
                                                      this->ncones_gain_, this->angle_exponent_,
                                                      this->distance_exponent_, this->cost_max_);
  auto path_smoother = std::make_shared<PathSmoothing>(this->smoothing_spline_order_,
                                                       this->smoothing_spline_coeffs_ratio_,
                                                       this->smoothing_spline_precision_);

  const std::vector<Cone *> recieved_cones;

  std::vector<Cone *> cone_array;
  for (const auto &cone : msg.cone_array) {
    RCLCPP_DEBUG(this->get_logger(), "[received] (%f, %f)\t%s", cone.position.x, cone.position.y,
                 cone.color.c_str());
    auto new_cone = new Cone(0, (float)cone.position.x, (float)cone.position.y);
    cone_array.push_back(new_cone);
  }

  cone_coloring->color_cones(cone_array, this->pose, 5);

  for (auto &cone : cone_coloring->current_left_cones) {
    track->add_cone_left(cone);
  }

  for (auto &cone : cone_coloring->current_right_cones) {
    track->add_cone_right(cone);
  }

  track->validateCones();  // Deal with cone outliers

  std::vector<PathPoint *> path = local_path_planner->processNewArray(track);  // Calculate Path

  path_smoother->defaultSmoother(path);

  publish_track_points(path);
}

void Planning::vehicle_localization_callback(const custom_interfaces::msg::VehicleState &msg) {
  this->pose = Pose(msg.position.x, msg.position.y, msg.theta);
}

/**
 * Publisher point by point
 */
void Planning::publish_track_points(const std::vector<PathPoint *> &path) const {
  auto message = custom_interfaces::msg::PathPointArray();
  for (auto const &element : path) {
    auto point = custom_interfaces::msg::PathPoint();
    point.x = element->getX();
    point.y = element->getY();
    point.v = element->getV();
    message.pathpoint_array.push_back(point);
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

void Planning::set_mission(Mission new_mission) { this->mission = new_mission; }

bool Planning::is_predicitve_mission() const {
  return this->mission == Mission::skidpad || this->mission == Mission::acceleration;
}