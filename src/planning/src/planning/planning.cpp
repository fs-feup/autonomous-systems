#include "planning/planning.hpp"

#include <vector>

#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"
#include "common_lib/config_load/config_load.hpp"

using std::placeholders::_1;

PlanningParameters Planning::load_config(std::string &adapter) {
  PlanningParameters params;
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("planning", "global", "global_config");
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Loading global config from: %s",
               global_config_path.c_str());
  YAML::Node global_config = YAML::LoadFile(global_config_path);

  adapter = global_config["global"]["adapter"].as<std::string>();
  params.using_simulated_se_ = global_config["global"]["use_simulated_se"].as<bool>();

  std::string planning_config_path =
      common_lib::config_load::get_config_yaml_path("planning", "planning", adapter);
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Loading planning config from: %s",
               planning_config_path.c_str());

  YAML::Node planning = YAML::LoadFile(planning_config_path);
  auto planning_config = planning["planning"];

  params.minimum_cone_distance_ = planning_config["minimum_cone_distance"].as<double>();
  params.maximum_cone_distance_ = planning_config["maximum_cone_distance"].as<double>();
  params.nc_angle_gain_ = planning_config["nc_angle_gain"].as<double>();
  params.nc_distance_gain_ = planning_config["nc_distance_gain"].as<double>();
  params.nc_angle_exponent_ = planning_config["nc_angle_exponent"].as<double>();
  params.nc_distance_exponent_ = planning_config["nc_distance_exponent"].as<double>();
  params.nc_max_cost_ = planning_config["nc_max_cost"].as<double>();
  params.nc_tolerance_ = planning_config["nc_tolerance"].as<double>();
  params.nc_search_depth_ = planning_config["nc_search_depth"].as<int>();
  params.nc_max_points_ = planning_config["nc_max_points"].as<int>();
  params.nc_lookback_points_ = planning_config["nc_lookback_points"].as<int>();
  params.nc_reset_global_path_ = planning_config["nc_reset_global_path"].as<int>();
  params.skidpad_tolerance_ = planning_config["skidpad_tolerance"].as<double>();
  params.skidpad_minimum_cones_ = planning_config["skidpad_minimum_cones"].as<int>();

  params.outliers_spline_order_ = planning_config["outliers_spline_order"].as<int>();
  params.outliers_spline_coeffs_ratio_ =
      planning_config["outliers_spline_coeffs_ratio"].as<float>();
  params.outliers_spline_precision_ = planning_config["outliers_spline_precision"].as<int>();
  params.smoothing_spline_order_ = planning_config["smoothing_spline_order"].as<int>();
  params.smoothing_spline_coeffs_ratio_ =
      planning_config["smoothing_spline_coeffs_ratio"].as<float>();
  params.smoothing_spline_precision_ = planning_config["smoothing_spline_precision"].as<int>();
  params.publishing_visualization_msgs_ =
      planning_config["publishing_visualization_msg"].as<bool>();
  params.desired_velocity_ = planning_config["desired_velocity"].as<double>();
  params.use_outlier_removal_ = planning_config["use_outlier_removal"].as<bool>();
  params.use_path_smoothing_ = planning_config["use_path_smoothing"].as<bool>();
  params.map_frame_id_ = adapter == "eufs" ? "base_footprint" : "map";
  params.minimum_velocity_ = planning_config["minimum_velocity"].as<double>();
  params.braking_acceleration_ = planning_config["braking_acceleration"].as<double>();
  params.normal_acceleration_ = planning_config["normal_acceleration"].as<double>();
  params.use_velocity_planning_ = planning_config["use_velocity_planning"].as<bool>();
  params.use_sliding_window_ = planning_config["use_sliding_window"].as<bool>();
  params.sliding_window_radius_ = planning_config["sliding_window_radius"].as<double>();

  return params;
}

Planning::Planning(const PlanningParameters &params)
    : Node("planning"),
      planning_config_(params),
      desired_velocity_(params.desired_velocity_),
      _map_frame_id_(params.map_frame_id_) {
  outliers_ = Outliers(planning_config_.outliers_);
  path_calculation_ = PathCalculation(planning_config_.path_calculation_);
  path_smoothing_ = PathSmoothing(planning_config_.smoothing_);
  velocity_planning_ = VelocityPlanning(planning_config_.velocity_planning_);

  param_client_ =
      this->create_client<rcl_interfaces::srv::GetParameters>("/pacsim/pacsim_node/get_parameters");
  fetch_discipline();

  // Control Publisher
  this->local_pub_ =
      this->create_publisher<custom_interfaces::msg::PathPointArray>("/path_planning/path", 10);


  // Publisher for execution time
  this->_planning_execution_time_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/path_planning/execution_time", 10);

  if (planning_config_.simulation_.publishing_visualization_msgs_) {
    // Publisher for visualization
    this->triangulations_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/path_planning/triangulations", 10);
    this->midpoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/midpoints", 10);
    this->visualization_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("/path_planning/smoothed_path", 10);

    this->full_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/full_path", 10);
    this->global_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/global_path", 10);
  }

  if (!planning_config_.simulation_.using_simulated_se_) {
    // Vehicle Localization Subscriber
    this->vl_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/state_estimation/vehicle_pose", 10,
        std::bind(&Planning::vehicle_localization_callback, this, _1));
    // State Estimation map Subscriber
    this->track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/state_estimation/map", 10, std::bind(&Planning::track_map_callback, this, _1));

    this->_lap_counter_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/state_estimation/lap_counter", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
          this->lap_counter_ = static_cast<int>(msg->data);
        });
  }

  RCLCPP_INFO(rclcpp::get_logger("planning"), "using simulated state estimation: %d",
              planning_config_.simulation_.using_simulated_se_);
}

void Planning::fetch_discipline() {
  common_lib::competition_logic::Mission mission_result =
      common_lib::competition_logic::Mission::NONE;

  if (!param_client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_ERROR(this->get_logger(), "Service /pacsim/pacsim_node/get_parameters not available.");
  } else {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("discipline");

    param_client_->async_send_request(
        request, [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
          auto response = future.get();
          common_lib::competition_logic::Mission mission_result =
              common_lib::competition_logic::Mission::AUTOCROSS;

          if (!response->values.empty() && response->values[0].type == 4) {  // Type 4 = string
            std::string discipline = response->values[0].string_value;
            RCLCPP_INFO(this->get_logger(), "Discipline received: %s", discipline.c_str());

            if (discipline == "skidpad") {
              mission_result = common_lib::competition_logic::Mission::SKIDPAD;
            } else if (discipline == "acceleration") {
              mission_result = common_lib::competition_logic::Mission::ACCELERATION;
            } else if (discipline == "trackdrive") {
              mission_result = common_lib::competition_logic::Mission::TRACKDRIVE;
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve discipline parameter.");
          }

          this->mission = mission_result;
        });
  }

  this->mission = mission_result;
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto number_of_cones_received = static_cast<int>(msg.cone_array.size());
  RCLCPP_DEBUG(this->get_logger(), "Planning received %i cones", number_of_cones_received);
  this->cone_array_ = common_lib::communication::cone_vector_from_custom_interfaces(msg);
  this->received_first_track_ = true;
  if (!(this->received_first_pose_)) {
    return;
  } else {
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

  std::vector<PathPoint> full_path = {};
  std::vector<PathPoint> final_path = {};
  std::vector<PathPoint> global_path_ = {};

  switch (this->mission) {
    case common_lib::competition_logic::Mission::NONE:
      RCLCPP_ERROR(this->get_logger(), "Mission is NONE, cannot run planning algorithms.");
      return;
    case common_lib::competition_logic::Mission::SKIDPAD:
      final_path = path_calculation_.skidpad_path(this->cone_array_, this->pose);
      break;

    case common_lib::competition_logic::Mission::ACCELERATION:
    case common_lib::competition_logic::Mission::EBS_TEST:
      full_path = path_calculation_.no_coloring_planning(this->cone_array_, this->pose);
      
      // Smooth the calculated path
      final_path = path_smoothing_.smooth_path(full_path, this->pose,
                                               this->initial_car_orientation_);
        
      {
        double dist_from_origin = sqrt(this->pose.position.x * this->pose.position.x +
                                       this->pose.position.y * this->pose.position.y);
        if (dist_from_origin > 90.0) {
           if (!braking_) {
            this->braking_ = true;
            this->brake_time_ = std::chrono::steady_clock::now();
           }
          for (auto &point : final_path) {
            std::chrono::duration<double> time_since_brake_start =  std::chrono::steady_clock::now() - this->brake_time_;
            point.ideal_velocity = std::max((desired_velocity_ + ( planning_config_.velocity_planning_.braking_acceleration_* time_since_brake_start.count())), 0.0);
          }
        } else {
          for (auto &point : final_path) {
            point.ideal_velocity = desired_velocity_;
          }
        }
      }
      
      break;

    case common_lib::competition_logic::Mission::AUTOCROSS:
      full_path = path_calculation_.no_coloring_planning(this->cone_array_, this->pose);
      final_path = path_smoothing_.smooth_path(full_path, this->pose,
                                                this->initial_car_orientation_);
      global_path_ = path_calculation_.get_global_path();
      velocity_planning_.trackdrive_velocity(final_path);
      if (this->lap_counter_ >= 1) {
        velocity_planning_.stop(final_path);
      }
      break;

    case common_lib::competition_logic::Mission::TRACKDRIVE:

      if (this->lap_counter_ == 0) {
        full_path = path_calculation_.no_coloring_planning(this->cone_array_, this->pose);
        final_path = path_smoothing_.smooth_path(full_path, this->pose,
                                                 this->initial_car_orientation_);
        global_path_ = path_calculation_.get_global_path();
        velocity_planning_.set_velocity(final_path);
      } else if (this->lap_counter_ >= 1 && this->lap_counter_ < 10) {
        if (!this->found_full_path_) {
          this->found_full_path_ = true;
          full_path =
              path_calculation_.calculate_trackdrive(this->cone_array_, this->pose);
          final_path = path_smoothing_.smooth_path(full_path, this->pose,
                                                   this->initial_car_orientation_);
          global_path_ = final_path;
          velocity_planning_.trackdrive_velocity(final_path);
          full_path_ = final_path;
        } else {
          // Use the full path for the next laps
          final_path = full_path_;
          global_path_ = full_path_;
        }
      } else if (this->lap_counter_ >= 10) {
        final_path = full_path_;
        global_path_ = full_path_;
        velocity_planning_.stop(final_path);
      }
      break;
    default:
      full_path = path_calculation_.no_coloring_planning(this->cone_array_, this->pose);
      final_path = path_smoothing_.smooth_path(full_path, this->pose,
                                               this->initial_car_orientation_);
      global_path_ = path_calculation_.get_global_path();
      velocity_planning_.set_velocity(final_path);
      break;
  }

  if (final_path.size() < 10) {
    RCLCPP_INFO(rclcpp::get_logger("planning"), "Final path size: %d",
                static_cast<int>(final_path.size()));
  }

  // Execution Time calculation
  rclcpp::Time end_time = this->now();
  std_msgs::msg::Float64 planning_execution_time;
  planning_execution_time.data = (end_time - start_time).seconds() * 1000;
  this->_planning_execution_time_publisher_->publish(planning_execution_time);
  publish_track_points(final_path);
  RCLCPP_DEBUG(this->get_logger(), "Planning will publish %i path points\n",
               static_cast<int>(final_path.size()));

  int errorcounter = 0;
  if (planning_config_.simulation_.publishing_visualization_msgs_) {
    std::vector<PathPoint> published_midpoints;
    publish_visualization_msgs(published_midpoints, full_path, final_path, global_path_);
  }
  if (errorcounter != 0) {
    RCLCPP_ERROR(this->get_logger(), "Number of midpoints with no close points: %d", errorcounter);
  }
}

void Planning::vehicle_localization_callback(const custom_interfaces::msg::Pose &msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received Pose: %lf - %lf - %lf", msg.x, msg.y, msg.theta);
  this->pose = Pose(msg.x, msg.y, msg.theta);

  if (!this->received_first_pose_) {
    this->initial_car_orientation_ = msg.theta;
  }
  if (this->received_first_track_ && !this->received_first_pose_) {
    run_planning_algorithms();
  }
  this->received_first_pose_ = true;
}

/**
 * Publisher point by point
 */
void Planning::publish_track_points(const std::vector<PathPoint> &path) const {
  auto message = common_lib::communication::custom_interfaces_array_from_vector(path);
  local_pub_->publish(message);
}

void Planning::set_mission(common_lib::competition_logic::Mission new_mission) {
  this->mission = new_mission;
}


void Planning::publish_visualization_msgs(const std::vector<PathPoint> &midPoints,
                                          const std::vector<PathPoint> &full_path,
                                          const std::vector<PathPoint> &final_path,
                                          const std::vector<PathPoint> &global_path) const {
  this->midpoints_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      midPoints, "midPoints", this->_map_frame_id_, "white"));
  this->triangulations_pub_->publish(common_lib::communication::lines_marker_from_triangulations(
    path_calculation_.triangulations, "triangulations", this->_map_frame_id_, 20, "white",0.05f,visualization_msgs::msg::Marker::MODIFY));

  this->full_path_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      full_path, "full_path", this->_map_frame_id_, "orange"));
  this->visualization_pub_->publish(common_lib::communication::line_marker_from_structure_array(
      final_path, "smoothed_path_planning", this->_map_frame_id_, 12, "green"));
  this->global_path_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      global_path, "global_path", this->_map_frame_id_, "blue", "cylinder", 0.8,
      visualization_msgs::msg::Marker::MODIFY));
}
