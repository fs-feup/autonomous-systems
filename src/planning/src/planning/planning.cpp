#include "planning/planning.hpp"

#include "adapter_planning/map.hpp"
#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"
#include "utils/message_converter.hpp"

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
  using_simulated_se_ = declare_parameter<int>("use_simulated_se", 0);
  publishing_visualization_msg_ = declare_parameter<int>("publishing_visualization_msg", 1);

  // Control Publisher
  this->local_pub_ =
      this->create_publisher<custom_interfaces::msg::PathPointArray>("/path_planning/path", 10);

  // Publisher for visualization
  this->visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/path_planning/smoothed_path", 10);

  // Publisher for visualization
  this->blue_cones_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_planning/blue_cones", 10);

  // Publisher for visualization
  this->yellow_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/path_planning/yellow_cones", 10);

  this->triangulations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/path_planning/triangulations", 10);

  // Publishes path from file in Skidpad & Acceleration events
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&Planning::publish_predicitive_track_points, this));
  RCLCPP_INFO(this->get_logger(), "Using %s adapter", mode.c_str());
  // Adapter to communicate with the car
  _adapter_ = adapter_map.at(mode)(std::shared_ptr<Planning>(this));

  if (!using_simulated_se_ || mode == "vehicle") {
    // State Estimation map Subscriber
    this->track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/state_estimation/map", 10, std::bind(&Planning::track_map_callback, this, _1));

    // Vehicle Localization Subscriber
    this->vl_sub_ = this->create_subscription<custom_interfaces::msg::VehicleState>(
        "/state_estimation/vehicle_state", 10,
        std::bind(&Planning::vehicle_localization_callback, this, _1));
  }
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray &msg) {
  RCLCPP_DEBUG(this->get_logger(), "Planning received %d cones", (int)msg.cone_array.size());
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

  std::vector<Cone *> cone_array = cone_vector_from_custom_interfaces(msg);

  cone_coloring->color_cones(cone_array, this->pose, 5);

  for (auto &cone : cone_coloring->current_left_cones) {
    track->add_cone_left(cone);
  }

  for (auto &cone : cone_coloring->current_right_cones) {
    track->add_cone_right(cone);
  }

  track->validateCones();  // Deal with cone outliers

  std::vector<PathPoint *> triangulations_path =
      local_path_planner->processNewArray(track);  // Calculate Path

  auto path = triangulations_path;
  path_smoother->defaultSmoother(path);

  RCLCPP_INFO(this->get_logger(), "Planning published %d path points", (int)path.size());
  publish_track_points(path);

  if (this->publishing_visualization_msg_) {
    publish_visualization_msgs(cone_coloring->current_left_cones,
                               cone_coloring->current_right_cones, triangulations_path, path);
  }
}

void Planning::vehicle_localization_callback(const custom_interfaces::msg::VehicleState &msg) {
  this->pose = Pose(msg.position.x, msg.position.y, msg.theta);
}

/**
 * Publisher point by point
 */
void Planning::publish_track_points(const std::vector<PathPoint *> &path) const {
  auto message = custom_interfaces_array_from_vector(path);
  local_pub_->publish(message);
}

void Planning::publish_predicitive_track_points() {
  // RCLCPP_INFO(this->get_logger(), "[mission] (%d)", this->mission);
  if (!this->is_predicitve_mission()) {
    return;
  }
  std::vector<PathPoint *> path = read_path_file(this->predictive_paths_[this->mission]);
  this->publish_track_points(path);
}

void Planning::set_mission(common_lib::competition_logic::Mission new_mission) {
  this->mission = new_mission;
}

bool Planning::is_predicitve_mission() const {
  return this->mission == common_lib::competition_logic::Mission::SKIDPAD ||
         this->mission == common_lib::competition_logic::Mission::ACCELERATION;
}

void Planning::publish_visualization_msgs(const std::vector<Cone *> &current_left_cones,
                                          const std::vector<Cone *> &current_right_cones,
                                          const std::vector<PathPoint *> &after_triangulations_path,
                                          const std::vector<PathPoint *> &final_path) {
  this->blue_cones_pub_->publish(
      marker_array_from_path_point_array(current_left_cones, "blue_cones_colored", "map", "blue"));
  this->yellow_cones_pub_->publish(marker_array_from_path_point_array(
      current_right_cones, "yellow_cones_colored", "map", "yellow"));
  this->visualization_pub_->publish(
      marker_array_from_path_point_array(final_path, "smoothed_path_planning", "map"));
  this->triangulations_pub_->publish(marker_array_from_path_point_array(
      after_triangulations_path, "after_triangulations_path", "map", "orange"));
}