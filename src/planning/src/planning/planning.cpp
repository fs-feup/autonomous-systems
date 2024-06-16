#include "planning/planning.hpp"

#include "adapter_planning/map.hpp"
#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"
#include "utils/message_converter.hpp"

using std::placeholders::_1;

Planning::Planning()
    : Node("planning"),
      mode(declare_parameter<std::string>("adapter", "pacsim")),
      angle_gain_(declare_parameter<double>("angle_gain", 3.7)),
      distance_gain_(declare_parameter<double>("distance_gain", 8.0)),
      ncones_gain_(declare_parameter<double>("ncones_gain", 8.7)),
      angle_exponent_(declare_parameter<double>("angle_exponent", 1)),
      distance_exponent_(declare_parameter<double>("distance_exponent", 1.7)),
      cost_max_(declare_parameter<double>("cost_max", 40)),
      outliers_spline_order_(declare_parameter<int>("outliers_spline_order_", 3)),
      outliers_spline_coeffs_ratio_(declare_parameter<float>("outliers_spline_coeffs_ratio", 3.0)),
      outliers_spline_precision_(declare_parameter<int>("outliers_spline_precision", 1)),
      smoothing_spline_order_(declare_parameter<int>("smoothing_spline_order", 3)),
      smoothing_spline_coeffs_ratio_(
          declare_parameter<float>("smoothing_spline_coeffs_ratio", 3.0)),
      smoothing_spline_precision_(declare_parameter<int>("smoothing_spline_precision", 10)),
      publishing_visualization_msgs_(declare_parameter<int>("publishing_visualization_msg", 1)),
      using_simulated_se_(declare_parameter<bool>("use_simulated_se", false)) {
  RCLCPP_DEBUG(this->get_logger(), "angle gain: %f; distance_gain : %f", angle_gain_,
               distance_gain_);

  cone_coloring = ConeColoring();
  outliers = Outliers();
  path_calculation = PathCalculation();
  path_smoothing = PathSmoothing();

  // Control Publisher
  this->local_pub_ =
      this->create_publisher<custom_interfaces::msg::PathPointArray>("/path_planning/path", 10);

  if (publishing_visualization_msgs_) {
    // Publisher for visualization
    this->visualization_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("/path_planning/smoothed_path", 10);

    // Publisher for visualization
    this->blue_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/blue_cones", 10);

    // Publisher for visualization
    this->yellow_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/yellow_cones", 10);

    this->triangulations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/triangulations", 10);
    // Publisher for visualization
    this->after_rem_blue_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/after_rem_blue_cones", 10);

    // Publisher for visualization
    this->after_rem_yellow_cones_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/path_planning/after_rem_yellow_cones", 10);
  }
  // Publishes path from file in Skidpad & Acceleration events
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&Planning::publish_predicitive_track_points, this));
  RCLCPP_INFO(this->get_logger(), "Using mode: %s", mode.c_str());
  // Adapter to communicate with the car
  _adapter_ = adapter_map.at(mode)((this));

  if (!using_simulated_se_ || mode == "vehicle") {
    // Vehicle Localization Subscriber
    this->vl_sub_ = this->create_subscription<custom_interfaces::msg::VehicleState>(
        "/state_estimation/vehicle_state", 10,
        std::bind(&Planning::vehicle_localization_callback, this, _1));
    // State Estimation map Subscriber
    this->track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/state_estimation/map", 10, std::bind(&Planning::track_map_callback, this, _1));
  }
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto number_of_cones_recieved = (int)msg.cone_array.size();
  RCLCPP_DEBUG(this->get_logger(), "Planning recieved %i cones", number_of_cones_recieved);
  this->cone_array_ = cone_vector_from_custom_interfaces(msg);
  this->recieved_first_track_ = true;
  if (this->is_predicitve_mission() || !(this->recieved_first_pose_)) {
    return;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Running all Planning algorithms");
    run_planning_algorithms();
  }
}

void Planning::run_planning_algorithms() {
  if (this->cone_array_.empty()) {
    publish_track_points({});
    return;
  }

  // Color the cones
  std::pair<std::vector<Cone>, std::vector<Cone>> colored_cones = cone_coloring.color_cones(this->cone_array_, this->pose);

  // Outliers dealt by approximating all cones
  std::pair<std::vector<Cone>, std::vector<Cone>> refined_colored_cones = outliers.approximate_cones_with_spline(colored_cones);
  // TODO: revert merge
  // Calculate middle points using triangulations
  std::vector<PathPoint> triangulations_path = path_calculation.process_delaunay_triangulations(refined_colored_cones);

  // Smooth the calculated path
  std::vector<PathPoint> final_path = path_smoothing.smooth_path(triangulations_path, this->pose);

  RCLCPP_DEBUG(this->get_logger(), "Planning published %i path points\n", (int)final_path.size());
  publish_track_points(final_path);

  if (this->publishing_visualization_msgs_) {
    publish_visualization_msgs(colored_cones.first, colored_cones.second,
      refined_colored_cones.first, refined_colored_cones.second,
      triangulations_path, final_path);
  }
}

void Planning::vehicle_localization_callback(const custom_interfaces::msg::VehicleState &msg) {
  this->pose = Pose(msg.position.x, msg.position.y, msg.theta);
  if (this->recieved_first_track_ && !this->recieved_first_pose_) {
    this->recieved_first_pose_ = true;
    run_planning_algorithms();
  } else {
    this->recieved_first_pose_ = true;
  }
}

/**
 * Publisher point by point
 */
void Planning::publish_track_points(const std::vector<PathPoint> &path) const {
  auto message = custom_interfaces_array_from_vector(path);
  local_pub_->publish(message);
}

void Planning::publish_predicitive_track_points() {
  // RCLCPP_INFO(this->get_logger(), "[mission] (%d)", this->mission);
  if (!this->is_predicitve_mission()) {
    return;
  }
  std::vector<PathPoint> path = read_path_file(this->predictive_paths_[this->mission]);
  this->publish_track_points(path);
}

void Planning::set_mission(common_lib::competition_logic::Mission new_mission) {
  this->mission = new_mission;
}

bool Planning::is_predicitve_mission() const {
  return this->mission == common_lib::competition_logic::Mission::SKIDPAD ||
         this->mission == common_lib::competition_logic::Mission::ACCELERATION;
}

void Planning::publish_visualization_msgs(const std::vector<Cone> &left_cones,
                                          const std::vector<Cone> &right_cones,
                                          const std::vector<Cone> &after_refining_blue_cones,
                                          const std::vector<Cone> &after_refining_yellow_cones,
                                          const std::vector<PathPoint> &after_triangulations_path,
                                          const std::vector<PathPoint> &final_path) {
  this->blue_cones_pub_->publish(
      marker_array_from_path_point_array(left_cones, "blue_cones_colored", "map", "blue"));
  this->yellow_cones_pub_->publish(
      marker_array_from_path_point_array(right_cones, "yellow_cones_colored", "map", "yellow"));
  this->after_rem_blue_cones_pub_->publish(marker_array_from_path_point_array(
      after_refining_blue_cones, "blue_cones_colored", "map", "blue"));
  this->after_rem_yellow_cones_pub_->publish(marker_array_from_path_point_array(
      after_refining_yellow_cones, "yellow_cones_colored", "map", "yellow"));
        this->triangulations_pub_->publish(marker_array_from_path_point_array(
      after_triangulations_path, "after_triangulations_path", "map", "orange"));
  this->visualization_pub_->publish(common_lib::communication::line_marker_from_structure_array(
      final_path, "smoothed_path_planning", "map", 12, "green"));
}