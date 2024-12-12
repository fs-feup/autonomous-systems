#include "planning/planning.hpp"

#include <vector>

#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"

using std::placeholders::_1;

Planning::Planning(const PlanningParameters &params)
    : Node("planning"),
      planning_config_(params),
      desired_velocity_(static_cast<double>(params.desired_velocity_)),
      _map_frame_id_(params.map_frame_id_) {
  cone_coloring_ = ConeColoring(planning_config_.cone_coloring_);
  outliers_ = Outliers(planning_config_.outliers_);
  path_calculation_ = PathCalculation(planning_config_.path_calculation_);
  path_smoothing_ = PathSmoothing(planning_config_.smoothing_);
  velocity_planning_ = VelocityPlanning(planning_config_.velocity_planning_);

  // Control Publisher
  this->local_pub_ =
      this->create_publisher<custom_interfaces::msg::PathPointArray>("/path_planning/path", 10);

  // Publisher for execution time
  this->_planning_execution_time_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/path_planning/execution_time", 10);

  if (planning_config_.simulation_.publishing_visualization_msgs_) {
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

  if (!planning_config_.simulation_.using_simulated_se_) {
    // Vehicle Localization Subscriber
    this->vl_sub_ = this->create_subscription<custom_interfaces::msg::VehicleState>(
        "/state_estimation/vehicle_state", 10,
        std::bind(&Planning::vehicle_localization_callback, this, _1));
    // State Estimation map Subscriber
    this->track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/state_estimation/map", 10, std::bind(&Planning::track_map_callback, this, _1));
  }
  RCLCPP_INFO(rclcpp::get_logger("planning"), "using simulated state estimation: %d",
              planning_config_.simulation_.using_simulated_se_);
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto number_of_cones_received = static_cast<int>(msg.cone_array.size());
  RCLCPP_DEBUG(this->get_logger(), "Planning received %i cones", number_of_cones_received);
  this->cone_array_ = common_lib::communication::cone_vector_from_custom_interfaces(msg);
  this->received_first_track_ = true;
  if (this->is_predicitve_mission() || !(this->received_first_pose_)) {
    return;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Running all Planning algorithms");
    run_planning_algorithms();
  }
}

void Planning::run_planning_algorithms() {
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Running Planning Algorithms");
  if (this->cone_array_.empty()) {
    publish_track_points({});
    return;
  }

  rclcpp::Time start_time = this->now();

  // Color the cones
  std::pair<std::vector<Cone>, std::vector<Cone>> colored_cones =
      cone_coloring_.color_cones(this->cone_array_, this->pose);
  if (colored_cones.first.size() < 2 || colored_cones.second.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("planning"), "Not enough cones to plan: %d blue, %d yellow",
                static_cast<int>(colored_cones.first.size()),
                static_cast<int>(colored_cones.second.size()));
    publish_track_points({});
    return;
  }

  // Outliers dealt by approximating all cones
  std::pair<std::vector<Cone>, std::vector<Cone>> refined_colored_cones =
      outliers_.approximate_cones_with_spline(colored_cones);
  if (refined_colored_cones.first.size() < 2 || refined_colored_cones.second.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("planning"),
                "Not enough cones to plan after outlier removal: %d blue, %d yellow",
                static_cast<int>(refined_colored_cones.first.size()),
                static_cast<int>(refined_colored_cones.second.size()));
    publish_track_points({});
    return;
  }
  for (auto &cone : colored_cones.first) {
    cone.color = Color::BLUE;
  }
  for (auto &cone : colored_cones.second) {
    cone.color = Color::YELLOW;
  }

  // Calculate middle points using triangulations
  std::vector<PathPoint> triangulations_path =
      path_calculation_.process_delaunay_triangulations(refined_colored_cones);
  if (triangulations_path.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("planning"),
                "Not enough cones to plan after triangulations: % d ",
                static_cast<int>(triangulations_path.size()));
    publish_track_points({});
    return;
  }

  // Smooth the calculated path
  std::vector<PathPoint> final_path =
      path_smoothing_.smooth_path(triangulations_path, this->pose, this->initial_car_orientation_);
  // -----------------------------------------

  if (true) {  // this->mission == common_lib::competition_logic::Mission::SKIDPAD
    if (!path_orientation_corrected_) { 
      // define the original predefined path
      double entryspeed = 6;
      double exitspeed = 10;
      double rotatingspeed = 5.5;
      std::vector<PathPoint> hardcoded_path_ = {PathPoint(0.000, 0, entryspeed),
PathPoint(0.500, 0, entryspeed),
PathPoint(1.000, 0, entryspeed),
PathPoint(1.500, 0, entryspeed),
PathPoint(2.000, 0, entryspeed),
PathPoint(2.500, 0, entryspeed),
PathPoint(3.000, 0, entryspeed),
PathPoint(3.500, 0, entryspeed),
PathPoint(4.000, 0, entryspeed),
PathPoint(4.500, 0, entryspeed),
PathPoint(5.000, 0, entryspeed),
PathPoint(5.500, 0, entryspeed),
PathPoint(6.000, 0, entryspeed),
PathPoint(6.500, 0, entryspeed),
PathPoint(7.000, 0, entryspeed),
PathPoint(7.500, 0, entryspeed),
PathPoint(8.000, 0, entryspeed),
PathPoint(8.500, 0, entryspeed),
PathPoint(9.000, 0, entryspeed),
PathPoint(9.500, 0, entryspeed),
PathPoint(10.000, 0, entryspeed),
PathPoint(10.500, 0, entryspeed),
PathPoint(11.000, 0, entryspeed),
PathPoint(11.500, 0, entryspeed),
PathPoint(12.000, 0, entryspeed),
PathPoint(12.500, 0, entryspeed),
PathPoint(13.000, 0, entryspeed),
PathPoint(13.500, 0, entryspeed),
PathPoint(14.000, 0, entryspeed),
PathPoint(14.500, 0, entryspeed),
PathPoint(15.000, -0.000, rotatingspeed),
PathPoint(15.573, -0.018, rotatingspeed),
PathPoint(16.144, -0.072, rotatingspeed),
PathPoint(16.710, -0.162, rotatingspeed),
PathPoint(17.269, -0.287, rotatingspeed),
PathPoint(17.820, -0.447, rotatingspeed),
PathPoint(18.359, -0.641, rotatingspeed),
PathPoint(18.885, -0.868, rotatingspeed),
PathPoint(19.396, -1.129, rotatingspeed),
PathPoint(19.889, -1.421, rotatingspeed),
PathPoint(20.364, -1.743, rotatingspeed),
PathPoint(20.816, -2.094, rotatingspeed),
PathPoint(21.246, -2.473, rotatingspeed),
PathPoint(21.652, -2.879, rotatingspeed),
PathPoint(22.031, -3.309, rotatingspeed),
PathPoint(22.382, -3.761, rotatingspeed),
PathPoint(22.704, -4.236, rotatingspeed),
PathPoint(22.996, -4.729, rotatingspeed),
PathPoint(23.257, -5.240, rotatingspeed),
PathPoint(23.484, -5.766, rotatingspeed),
PathPoint(23.678, -6.305, rotatingspeed),
PathPoint(23.838, -6.856, rotatingspeed),
PathPoint(23.963, -7.415, rotatingspeed),
PathPoint(24.053, -7.981, rotatingspeed),
PathPoint(24.107, -8.552, rotatingspeed),
PathPoint(24.125, -9.125, rotatingspeed),
PathPoint(24.107, -9.698, rotatingspeed),
PathPoint(24.053, -10.269, rotatingspeed),
PathPoint(23.963, -10.835, rotatingspeed),
PathPoint(23.838, -11.394, rotatingspeed),
PathPoint(23.678, -11.945, rotatingspeed),
PathPoint(23.484, -12.484, rotatingspeed),
PathPoint(23.257, -13.010, rotatingspeed),
PathPoint(22.996, -13.521, rotatingspeed),
PathPoint(22.704, -14.014, rotatingspeed),
PathPoint(22.382, -14.489, rotatingspeed),
PathPoint(22.031, -14.941, rotatingspeed),
PathPoint(21.652, -15.371, rotatingspeed),
PathPoint(21.246, -15.777, rotatingspeed),
PathPoint(20.816, -16.156, rotatingspeed),
PathPoint(20.364, -16.507, rotatingspeed),
PathPoint(19.889, -16.829, rotatingspeed),
PathPoint(19.396, -17.121, rotatingspeed),
PathPoint(18.885, -17.382, rotatingspeed),
PathPoint(18.359, -17.609, rotatingspeed),
PathPoint(17.820, -17.803, rotatingspeed),
PathPoint(17.269, -17.963, rotatingspeed),
PathPoint(16.710, -18.088, rotatingspeed),
PathPoint(16.144, -18.178, rotatingspeed),
PathPoint(15.573, -18.232, rotatingspeed),
PathPoint(15.000, -18.250, rotatingspeed),
PathPoint(14.427, -18.232, rotatingspeed),
PathPoint(13.856, -18.178, rotatingspeed),
PathPoint(13.290, -18.088, rotatingspeed),
PathPoint(12.731, -17.963, rotatingspeed),
PathPoint(12.180, -17.803, rotatingspeed),
PathPoint(11.641, -17.609, rotatingspeed),
PathPoint(11.115, -17.382, rotatingspeed),
PathPoint(10.604, -17.121, rotatingspeed),
PathPoint(10.111, -16.829, rotatingspeed),
PathPoint(9.636, -16.507, rotatingspeed),
PathPoint(9.184, -16.156, rotatingspeed),
PathPoint(8.754, -15.777, rotatingspeed),
PathPoint(8.348, -15.371, rotatingspeed),
PathPoint(7.969, -14.941, rotatingspeed),
PathPoint(7.618, -14.489, rotatingspeed),
PathPoint(7.296, -14.014, rotatingspeed),
PathPoint(7.004, -13.521, rotatingspeed),
PathPoint(6.743, -13.010, rotatingspeed),
PathPoint(6.516, -12.484, rotatingspeed),
PathPoint(6.322, -11.945, rotatingspeed),
PathPoint(6.162, -11.394, rotatingspeed),
PathPoint(6.037, -10.835, rotatingspeed),
PathPoint(5.947, -10.269, rotatingspeed),
PathPoint(5.893, -9.698, rotatingspeed),
PathPoint(5.875, -9.125, rotatingspeed),
PathPoint(5.893, -8.552, rotatingspeed),
PathPoint(5.947, -7.981, rotatingspeed),
PathPoint(6.037, -7.415, rotatingspeed),
PathPoint(6.162, -6.856, rotatingspeed),
PathPoint(6.322, -6.305, rotatingspeed),
PathPoint(6.516, -5.766, rotatingspeed),
PathPoint(6.743, -5.240, rotatingspeed),
PathPoint(7.004, -4.729, rotatingspeed),
PathPoint(7.296, -4.236, rotatingspeed),
PathPoint(7.618, -3.761, rotatingspeed),
PathPoint(7.969, -3.309, rotatingspeed),
PathPoint(8.348, -2.879, rotatingspeed),
PathPoint(8.754, -2.473, rotatingspeed),
PathPoint(9.184, -2.094, rotatingspeed),
PathPoint(9.636, -1.743, rotatingspeed),
PathPoint(10.111, -1.421, rotatingspeed),
PathPoint(10.604, -1.129, rotatingspeed),
PathPoint(11.115, -0.868, rotatingspeed),
PathPoint(11.641, -0.641, rotatingspeed),
PathPoint(12.180, -0.447, rotatingspeed),
PathPoint(12.731, -0.287, rotatingspeed),
PathPoint(13.290, -0.162, rotatingspeed),
PathPoint(13.856, -0.072, rotatingspeed),
PathPoint(14.427, -0.018, rotatingspeed),
PathPoint(15.000, -0.000, rotatingspeed),
PathPoint(15.573, -0.018, rotatingspeed),
PathPoint(16.144, -0.072, rotatingspeed),
PathPoint(16.710, -0.162, rotatingspeed),
PathPoint(17.269, -0.287, rotatingspeed),
PathPoint(17.820, -0.447, rotatingspeed),
PathPoint(18.359, -0.641, rotatingspeed),
PathPoint(18.885, -0.868, rotatingspeed),
PathPoint(19.396, -1.129, rotatingspeed),
PathPoint(19.889, -1.421, rotatingspeed),
PathPoint(20.364, -1.743, rotatingspeed),
PathPoint(20.816, -2.094, rotatingspeed),
PathPoint(21.246, -2.473, rotatingspeed),
PathPoint(21.652, -2.879, rotatingspeed),
PathPoint(22.031, -3.309, rotatingspeed),
PathPoint(22.382, -3.761, rotatingspeed),
PathPoint(22.704, -4.236, rotatingspeed),
PathPoint(22.996, -4.729, rotatingspeed),
PathPoint(23.257, -5.240, rotatingspeed),
PathPoint(23.484, -5.766, rotatingspeed),
PathPoint(23.678, -6.305, rotatingspeed),
PathPoint(23.838, -6.856, rotatingspeed),
PathPoint(23.963, -7.415, rotatingspeed),
PathPoint(24.053, -7.981, rotatingspeed),
PathPoint(24.107, -8.552, rotatingspeed),
PathPoint(24.125, -9.125, rotatingspeed),
PathPoint(24.107, -9.698, rotatingspeed),
PathPoint(24.053, -10.269, rotatingspeed),
PathPoint(23.963, -10.835, rotatingspeed),
PathPoint(23.838, -11.394, rotatingspeed),
PathPoint(23.678, -11.945, rotatingspeed),
PathPoint(23.484, -12.484, rotatingspeed),
PathPoint(23.257, -13.010, rotatingspeed),
PathPoint(22.996, -13.521, rotatingspeed),
PathPoint(22.704, -14.014, rotatingspeed),
PathPoint(22.382, -14.489, rotatingspeed),
PathPoint(22.031, -14.941, rotatingspeed),
PathPoint(21.652, -15.371, rotatingspeed),
PathPoint(21.246, -15.777, rotatingspeed),
PathPoint(20.816, -16.156, rotatingspeed),
PathPoint(20.364, -16.507, rotatingspeed),
PathPoint(19.889, -16.829, rotatingspeed),
PathPoint(19.396, -17.121, rotatingspeed),
PathPoint(18.885, -17.382, rotatingspeed),
PathPoint(18.359, -17.609, rotatingspeed),
PathPoint(17.820, -17.803, rotatingspeed),
PathPoint(17.269, -17.963, rotatingspeed),
PathPoint(16.710, -18.088, rotatingspeed),
PathPoint(16.144, -18.178, rotatingspeed),
PathPoint(15.573, -18.232, rotatingspeed),
PathPoint(15.000, -18.250, rotatingspeed),
PathPoint(14.427, -18.232, rotatingspeed),
PathPoint(13.856, -18.178, rotatingspeed),
PathPoint(13.290, -18.088, rotatingspeed),
PathPoint(12.731, -17.963, rotatingspeed),
PathPoint(12.180, -17.803, rotatingspeed),
PathPoint(11.641, -17.609, rotatingspeed),
PathPoint(11.115, -17.382, rotatingspeed),
PathPoint(10.604, -17.121, rotatingspeed),
PathPoint(10.111, -16.829, rotatingspeed),
PathPoint(9.636, -16.507, rotatingspeed),
PathPoint(9.184, -16.156, rotatingspeed),
PathPoint(8.754, -15.777, rotatingspeed),
PathPoint(8.348, -15.371, rotatingspeed),
PathPoint(7.969, -14.941, rotatingspeed),
PathPoint(7.618, -14.489, rotatingspeed),
PathPoint(7.296, -14.014, rotatingspeed),
PathPoint(7.004, -13.521, rotatingspeed),
PathPoint(6.743, -13.010, rotatingspeed),
PathPoint(6.516, -12.484, rotatingspeed),
PathPoint(6.322, -11.945, rotatingspeed),
PathPoint(6.162, -11.394, rotatingspeed),
PathPoint(6.037, -10.835, rotatingspeed),
PathPoint(5.947, -10.269, rotatingspeed),
PathPoint(5.893, -9.698, rotatingspeed),
PathPoint(5.875, -9.125, rotatingspeed),
PathPoint(5.893, -8.552, rotatingspeed),
PathPoint(5.947, -7.981, rotatingspeed),
PathPoint(6.037, -7.415, rotatingspeed),
PathPoint(6.162, -6.856, rotatingspeed),
PathPoint(6.322, -6.305, rotatingspeed),
PathPoint(6.516, -5.766, rotatingspeed),
PathPoint(6.743, -5.240, rotatingspeed),
PathPoint(7.004, -4.729, rotatingspeed),
PathPoint(7.296, -4.236, rotatingspeed),
PathPoint(7.618, -3.761, rotatingspeed),
PathPoint(7.969, -3.309, rotatingspeed),
PathPoint(8.348, -2.879, rotatingspeed),
PathPoint(8.754, -2.473, rotatingspeed),
PathPoint(9.184, -2.094, rotatingspeed),
PathPoint(9.636, -1.743, rotatingspeed),
PathPoint(10.111, -1.421, rotatingspeed),
PathPoint(10.604, -1.129, rotatingspeed),
PathPoint(11.115, -0.868, rotatingspeed),
PathPoint(11.641, -0.641, rotatingspeed),
PathPoint(12.180, -0.447, rotatingspeed),
PathPoint(12.731, -0.287, rotatingspeed),
PathPoint(13.290, -0.162, rotatingspeed),
PathPoint(13.856, -0.072, rotatingspeed),
PathPoint(14.427, -0.018, rotatingspeed),
PathPoint(15.000, 0.000, rotatingspeed),
PathPoint(15.573, 0.018, rotatingspeed),
PathPoint(16.144, 0.072, rotatingspeed),
PathPoint(16.710, 0.162, rotatingspeed),
PathPoint(17.269, 0.287, rotatingspeed),
PathPoint(17.820, 0.447, rotatingspeed),
PathPoint(18.359, 0.641, rotatingspeed),
PathPoint(18.885, 0.868, rotatingspeed),
PathPoint(19.396, 1.129, rotatingspeed),
PathPoint(19.889, 1.421, rotatingspeed),
PathPoint(20.364, 1.743, rotatingspeed),
PathPoint(20.816, 2.094, rotatingspeed),
PathPoint(21.246, 2.473, rotatingspeed),
PathPoint(21.652, 2.879, rotatingspeed),
PathPoint(22.031, 3.309, rotatingspeed),
PathPoint(22.382, 3.761, rotatingspeed),
PathPoint(22.704, 4.236, rotatingspeed),
PathPoint(22.996, 4.729, rotatingspeed),
PathPoint(23.257, 5.240, rotatingspeed),
PathPoint(23.484, 5.766, rotatingspeed),
PathPoint(23.678, 6.305, rotatingspeed),
PathPoint(23.838, 6.856, rotatingspeed),
PathPoint(23.963, 7.415, rotatingspeed),
PathPoint(24.053, 7.981, rotatingspeed),
PathPoint(24.107, 8.552, rotatingspeed),
PathPoint(24.125, 9.125, rotatingspeed),
PathPoint(24.107, 9.698, rotatingspeed),
PathPoint(24.053, 10.269, rotatingspeed),
PathPoint(23.963, 10.835, rotatingspeed),
PathPoint(23.838, 11.394, rotatingspeed),
PathPoint(23.678, 11.945, rotatingspeed),
PathPoint(23.484, 12.484, rotatingspeed),
PathPoint(23.257, 13.010, rotatingspeed),
PathPoint(22.996, 13.521, rotatingspeed),
PathPoint(22.704, 14.014, rotatingspeed),
PathPoint(22.382, 14.489, rotatingspeed),
PathPoint(22.031, 14.941, rotatingspeed),
PathPoint(21.652, 15.371, rotatingspeed),
PathPoint(21.246, 15.777, rotatingspeed),
PathPoint(20.816, 16.156, rotatingspeed),
PathPoint(20.364, 16.507, rotatingspeed),
PathPoint(19.889, 16.829, rotatingspeed),
PathPoint(19.396, 17.121, rotatingspeed),
PathPoint(18.885, 17.382, rotatingspeed),
PathPoint(18.359, 17.609, rotatingspeed),
PathPoint(17.820, 17.803, rotatingspeed),
PathPoint(17.269, 17.963, rotatingspeed),
PathPoint(16.710, 18.088, rotatingspeed),
PathPoint(16.144, 18.178, rotatingspeed),
PathPoint(15.573, 18.232, rotatingspeed),
PathPoint(15.000, 18.250, rotatingspeed),
PathPoint(14.427, 18.232, rotatingspeed),
PathPoint(13.856, 18.178, rotatingspeed),
PathPoint(13.290, 18.088, rotatingspeed),
PathPoint(12.731, 17.963, rotatingspeed),
PathPoint(12.180, 17.803, rotatingspeed),
PathPoint(11.641, 17.609, rotatingspeed),
PathPoint(11.115, 17.382, rotatingspeed),
PathPoint(10.604, 17.121, rotatingspeed),
PathPoint(10.111, 16.829, rotatingspeed),
PathPoint(9.636, 16.507, rotatingspeed),
PathPoint(9.184, 16.156, rotatingspeed),
PathPoint(8.754, 15.777, rotatingspeed),
PathPoint(8.348, 15.371, rotatingspeed),
PathPoint(7.969, 14.941, rotatingspeed),
PathPoint(7.618, 14.489, rotatingspeed),
PathPoint(7.296, 14.014, rotatingspeed),
PathPoint(7.004, 13.521, rotatingspeed),
PathPoint(6.743, 13.010, rotatingspeed),
PathPoint(6.516, 12.484, rotatingspeed),
PathPoint(6.322, 11.945, rotatingspeed),
PathPoint(6.162, 11.394, rotatingspeed),
PathPoint(6.037, 10.835, rotatingspeed),
PathPoint(5.947, 10.269, rotatingspeed),
PathPoint(5.893, 9.698, rotatingspeed),
PathPoint(5.875, 9.125, rotatingspeed),
PathPoint(5.893, 8.552, rotatingspeed),
PathPoint(5.947, 7.981, rotatingspeed),
PathPoint(6.037, 7.415, rotatingspeed),
PathPoint(6.162, 6.856, rotatingspeed),
PathPoint(6.322, 6.305, rotatingspeed),
PathPoint(6.516, 5.766, rotatingspeed),
PathPoint(6.743, 5.240, rotatingspeed),
PathPoint(7.004, 4.729, rotatingspeed),
PathPoint(7.296, 4.236, rotatingspeed),
PathPoint(7.618, 3.761, rotatingspeed),
PathPoint(7.969, 3.309, rotatingspeed),
PathPoint(8.348, 2.879, rotatingspeed),
PathPoint(8.754, 2.473, rotatingspeed),
PathPoint(9.184, 2.094, rotatingspeed),
PathPoint(9.636, 1.743, rotatingspeed),
PathPoint(10.111, 1.421, rotatingspeed),
PathPoint(10.604, 1.129, rotatingspeed),
PathPoint(11.115, 0.868, rotatingspeed),
PathPoint(11.641, 0.641, rotatingspeed),
PathPoint(12.180, 0.447, rotatingspeed),
PathPoint(12.731, 0.287, rotatingspeed),
PathPoint(13.290, 0.162, rotatingspeed),
PathPoint(13.856, 0.072, rotatingspeed),
PathPoint(14.427, 0.018, rotatingspeed),
PathPoint(15.000, 0.000, rotatingspeed),
PathPoint(15.573, 0.018, rotatingspeed),
PathPoint(16.144, 0.072, rotatingspeed),
PathPoint(16.710, 0.162, rotatingspeed),
PathPoint(17.269, 0.287, rotatingspeed),
PathPoint(17.820, 0.447, rotatingspeed),
PathPoint(18.359, 0.641, rotatingspeed),
PathPoint(18.885, 0.868, rotatingspeed),
PathPoint(19.396, 1.129, rotatingspeed),
PathPoint(19.889, 1.421, rotatingspeed),
PathPoint(20.364, 1.743, rotatingspeed),
PathPoint(20.816, 2.094, rotatingspeed),
PathPoint(21.246, 2.473, rotatingspeed),
PathPoint(21.652, 2.879, rotatingspeed),
PathPoint(22.031, 3.309, rotatingspeed),
PathPoint(22.382, 3.761, rotatingspeed),
PathPoint(22.704, 4.236, rotatingspeed),
PathPoint(22.996, 4.729, rotatingspeed),
PathPoint(23.257, 5.240, rotatingspeed),
PathPoint(23.484, 5.766, rotatingspeed),
PathPoint(23.678, 6.305, rotatingspeed),
PathPoint(23.838, 6.856, rotatingspeed),
PathPoint(23.963, 7.415, rotatingspeed),
PathPoint(24.053, 7.981, rotatingspeed),
PathPoint(24.107, 8.552, rotatingspeed),
PathPoint(24.125, 9.125, rotatingspeed),
PathPoint(24.107, 9.698, rotatingspeed),
PathPoint(24.053, 10.269, rotatingspeed),
PathPoint(23.963, 10.835, rotatingspeed),
PathPoint(23.838, 11.394, rotatingspeed),
PathPoint(23.678, 11.945, rotatingspeed),
PathPoint(23.484, 12.484, rotatingspeed),
PathPoint(23.257, 13.010, rotatingspeed),
PathPoint(22.996, 13.521, rotatingspeed),
PathPoint(22.704, 14.014, rotatingspeed),
PathPoint(22.382, 14.489, rotatingspeed),
PathPoint(22.031, 14.941, rotatingspeed),
PathPoint(21.652, 15.371, rotatingspeed),
PathPoint(21.246, 15.777, rotatingspeed),
PathPoint(20.816, 16.156, rotatingspeed),
PathPoint(20.364, 16.507, rotatingspeed),
PathPoint(19.889, 16.829, rotatingspeed),
PathPoint(19.396, 17.121, rotatingspeed),
PathPoint(18.885, 17.382, rotatingspeed),
PathPoint(18.359, 17.609, rotatingspeed),
PathPoint(17.820, 17.803, rotatingspeed),
PathPoint(17.269, 17.963, rotatingspeed),
PathPoint(16.710, 18.088, rotatingspeed),
PathPoint(16.144, 18.178, rotatingspeed),
PathPoint(15.573, 18.232, rotatingspeed),
PathPoint(15.000, 18.250, rotatingspeed),
PathPoint(14.427, 18.232, rotatingspeed),
PathPoint(13.856, 18.178, rotatingspeed),
PathPoint(13.290, 18.088, rotatingspeed),
PathPoint(12.731, 17.963, rotatingspeed),
PathPoint(12.180, 17.803, rotatingspeed),
PathPoint(11.641, 17.609, rotatingspeed),
PathPoint(11.115, 17.382, rotatingspeed),
PathPoint(10.604, 17.121, rotatingspeed),
PathPoint(10.111, 16.829, rotatingspeed),
PathPoint(9.636, 16.507, rotatingspeed),
PathPoint(9.184, 16.156, rotatingspeed),
PathPoint(8.754, 15.777, rotatingspeed),
PathPoint(8.348, 15.371, rotatingspeed),
PathPoint(7.969, 14.941, rotatingspeed),
PathPoint(7.618, 14.489, rotatingspeed),
PathPoint(7.296, 14.014, rotatingspeed),
PathPoint(7.004, 13.521, rotatingspeed),
PathPoint(6.743, 13.010, rotatingspeed),
PathPoint(6.516, 12.484, rotatingspeed),
PathPoint(6.322, 11.945, rotatingspeed),
PathPoint(6.162, 11.394, rotatingspeed),
PathPoint(6.037, 10.835, rotatingspeed),
PathPoint(5.947, 10.269, rotatingspeed),
PathPoint(5.893, 9.698, rotatingspeed),
PathPoint(5.875, 9.125, rotatingspeed),
PathPoint(5.893, 8.552, rotatingspeed),
PathPoint(5.947, 7.981, rotatingspeed),
PathPoint(6.037, 7.415, rotatingspeed),
PathPoint(6.162, 6.856, rotatingspeed),
PathPoint(6.322, 6.305, rotatingspeed),
PathPoint(6.516, 5.766, rotatingspeed),
PathPoint(6.743, 5.240, rotatingspeed),
PathPoint(7.004, 4.729, rotatingspeed),
PathPoint(7.296, 4.236, rotatingspeed),
PathPoint(7.618, 3.761, rotatingspeed),
PathPoint(7.969, 3.309, rotatingspeed),
PathPoint(8.348, 2.879, rotatingspeed),
PathPoint(8.754, 2.473, rotatingspeed),
PathPoint(9.184, 2.094, rotatingspeed),
PathPoint(9.636, 1.743, rotatingspeed),
PathPoint(10.111, 1.421, rotatingspeed),
PathPoint(10.604, 1.129, rotatingspeed),
PathPoint(11.115, 0.868, rotatingspeed),
PathPoint(11.641, 0.641, rotatingspeed),
PathPoint(12.180, 0.447, rotatingspeed),
PathPoint(12.731, 0.287, rotatingspeed),
PathPoint(13.290, 0.162, rotatingspeed),
PathPoint(13.856, 0.072, rotatingspeed),
PathPoint(14.427, 0.018, rotatingspeed),
PathPoint(15.000, 0, exitspeed),
PathPoint(15.500, 0, exitspeed),
PathPoint(16.000, 0, exitspeed),
PathPoint(16.500, 0, exitspeed),
PathPoint(17.000, 0, exitspeed),
PathPoint(17.500, 0, exitspeed),
PathPoint(18.000, 0, exitspeed),
PathPoint(18.500, 0, exitspeed),
PathPoint(19.000, 0, exitspeed),
PathPoint(19.500, 0, exitspeed),
PathPoint(20.000, 0, exitspeed),
PathPoint(20.500, 0, exitspeed),
PathPoint(21.000, 0, exitspeed),
PathPoint(21.500, 0, exitspeed),
PathPoint(22.000, 0, exitspeed),
PathPoint(22.500, 0, exitspeed),
PathPoint(23.000, 0, exitspeed),
PathPoint(23.500, 0, exitspeed),
PathPoint(24.000, 0, exitspeed),
PathPoint(24.500, 0, exitspeed),
PathPoint(25.000, 0, 0),
PathPoint(25.500, 0, 0),
PathPoint(26.000, 0, 0),
PathPoint(26.500, 0, 0),
PathPoint(27.000, 0, 0),
PathPoint(27.500, 0, 0),
PathPoint(28.000, 0, 0),
PathPoint(28.500, 0, 0),
PathPoint(29.000, 0, 0),
PathPoint(29.500, 0, 0),
PathPoint(30.000, 0, 0),
PathPoint(30.500, 0, 0),
PathPoint(31.000, 0, 0),
PathPoint(31.500, 0, 0),
PathPoint(32.000, 0, 0),
PathPoint(32.500, 0, 0),
PathPoint(33.000, 0, 0),
PathPoint(33.500, 0, 0),
PathPoint(34.000, 0, 0),
PathPoint(34.500, 0, 0),
PathPoint(35.000, 0, 0),
PathPoint(35.500, 0, 0),
PathPoint(36.000, 0, 0),
PathPoint(36.500, 0, 0),
PathPoint(37.000, 0, 0),
PathPoint(37.500, 0, 0),
PathPoint(38.000, 0, 0),
PathPoint(38.500, 0, 0),
PathPoint(39.000, 0, 0),
PathPoint(39.500, 0, 0),
PathPoint(40.000, 0, 0),
PathPoint(40.500, 0, 0),
PathPoint(41.000, 0, 0),
PathPoint(41.500, 0, 0),
PathPoint(42.000, 0, 0),
PathPoint(42.500, 0, 0),
PathPoint(43.000, 0, 0),
PathPoint(43.500, 0, 0),
PathPoint(44.000, 0, 0),
PathPoint(44.500, 0, 0),};


      // sort the cones by distance
      sort(cone_array_.begin(), cone_array_.end(), [this](const Cone &a, const Cone &b) {
        return sqrt(pow((a.position.x - this->pose.position.x), 2) +
                    pow((a.position.y - this->pose.position.y), 2)) <
               sqrt(pow((b.position.x - this->pose.position.x), 2) +
                    pow((b.position.y - this->pose.position.y), 2));
      });

      // get the middle point between the two closest points
      PathPoint middle_closest =
          PathPoint((cone_array_[0].position.x + cone_array_[1].position.x) / 2,
                    (cone_array_[0].position.y + cone_array_[1].position.y) / 2, 0);

      // get the middle point between the two farthest points
      PathPoint middle_farthest =
          PathPoint((cone_array_[2].position.x + cone_array_[3].position.x) / 2,
                    (cone_array_[2].position.y + cone_array_[3].position.y) / 2, 0);

      // find the slope of the line between the two middle points
      double slope = (middle_farthest.position.y - middle_closest.position.y) /
                     (middle_farthest.position.x + 0.0001 -
                      middle_closest.position.x);  // 0.0001 to avoid division by zero, should not
                                                   // happen but this is a safety measure

      // calculate the angle
      double angle = atan(slope);

      // rotate all the points by the angle and add the origin point
      for (auto &point : hardcoded_path_) {
        double x = point.position.x;
        double y = point.position.y;
        point.position.x = x * cos(angle) - y * sin(angle) + middle_closest.position.x;
        point.position.y = x * sin(angle) + y * cos(angle) + middle_closest.position.y;
      }

      this->predefined_path_ = hardcoded_path_;
      path_orientation_corrected_ = true;
    }

    while (!predefined_path_.empty() &&
           this->pose.position.euclidean_distance(predefined_path_[0].position) < 1) {
      predefined_path_.erase(predefined_path_.begin());
    }

    // set it as the final path
    final_path = std::vector<PathPoint>(predefined_path_.begin(), predefined_path_.begin() + 40);
  }

  if (final_path.size() < 10) {
    RCLCPP_INFO(rclcpp::get_logger("planning"), "Final path size: %d",
                static_cast<int>(final_path.size()));
  }

  if (this->mission == common_lib::competition_logic::Mission::ACCELERATION) {  // change later
    double dist_from_origin = sqrt(this->pose.position.x * this->pose.position.x +
                                   this->pose.position.y * this->pose.position.y);
    if (dist_from_origin > 80.0) {
      for (auto &point : final_path) {
        point.ideal_velocity = 0.0;
      }
    } else {
      for (auto &point : final_path) {
        point.ideal_velocity = 300.0;
      }
    }
  } else {
    //velocity_planning_.set_velocity(final_path);
  }

  // Execution Time calculation
  rclcpp::Time end_time = this->now();
  std_msgs::msg::Float64 planning_execution_time;
  planning_execution_time.data = (end_time - start_time).seconds() * 1000;
  this->_planning_execution_time_publisher_->publish(planning_execution_time);

  publish_track_points(final_path);
  RCLCPP_DEBUG(this->get_logger(), "Planning will publish %i path points\n",
               static_cast<int>(final_path.size()));

  if (planning_config_.simulation_.publishing_visualization_msgs_) {
    publish_visualization_msgs(colored_cones.first, colored_cones.second,
                               refined_colored_cones.first, refined_colored_cones.second,
                               triangulations_path, final_path);
  }
}

void Planning::vehicle_localization_callback(const custom_interfaces::msg::VehicleState &msg) {
  this->pose = Pose(msg.position.x, msg.position.y, msg.theta);

  if (!this->received_first_pose_) {
    this->initial_car_orientation_ = msg.theta;
  }
  if (this->received_first_track_ && !this->received_first_pose_) {
    this->received_first_pose_ = true;
    run_planning_algorithms();
  } else {
    this->received_first_pose_ = true;
  }
}

/**
 * Publisher point by point
 */
void Planning::publish_track_points(const std::vector<PathPoint> &path) const {
  auto message = common_lib::communication::custom_interfaces_array_from_vector(path);

  local_pub_->publish(message);
}

void Planning::publish_predicitive_track_points() {
  // RCLCPP_INFO(this->get_logger(), "[mission] (%d)", this->mission);
  if (!this->is_predicitve_mission()) {
    return;
  }
  std::vector<PathPoint> path = read_path_file(this->predictive_paths_[this->mission]);

  // TODO: Remove this when velocity planning is a reality
  for (auto &path_point : path) {
    path_point.ideal_velocity = desired_velocity_;
  }

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
                                          const std::vector<PathPoint> &final_path) const {
  this->blue_cones_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      left_cones, "blue_cones_colored", this->_map_frame_id_, "blue"));
  this->yellow_cones_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      right_cones, "yellow_cones_colored", this->_map_frame_id_, "yellow"));
  this->after_rem_blue_cones_pub_->publish(
      common_lib::communication::marker_array_from_structure_array(
          after_refining_blue_cones, "blue_cones_colored", this->_map_frame_id_, "blue"));
  this->after_rem_yellow_cones_pub_->publish(
      common_lib::communication::marker_array_from_structure_array(
          after_refining_yellow_cones, "yellow_cones_colored", this->_map_frame_id_, "yellow"));
  this->triangulations_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      after_triangulations_path, "after_triangulations_path", this->_map_frame_id_, "orange"));
  this->visualization_pub_->publish(common_lib::communication::line_marker_from_structure_array(
      final_path, "smoothed_path_planning", this->_map_frame_id_, 12, "green"));
}