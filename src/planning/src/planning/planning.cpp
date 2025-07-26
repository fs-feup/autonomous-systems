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
  params.projected_point_distance_ = planning_config["projected_point_distance"].as<double>();
  params.nc_angle_gain_ = planning_config["nc_angle_gain"].as<double>();
  params.nc_distance_gain_ = planning_config["nc_distance_gain"].as<double>();
  params.nc_angle_exponent_ = planning_config["nc_angle_exponent"].as<double>();
  params.nc_distance_exponent_ = planning_config["nc_distance_exponent"].as<double>();
  params.nc_max_cost_ = planning_config["nc_max_cost"].as<double>();
  params.nc_search_depth_ = planning_config["nc_search_depth"].as<int>();
  params.nc_max_points_ = planning_config["nc_max_points"].as<int>();

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
  params.desired_velocity_ = planning_config["pre_defined_velocity_planning"].as<double>();
  params.use_outlier_removal_ = planning_config["use_outlier_removal"].as<bool>();
  params.use_path_smoothing_ = planning_config["use_path_smoothing"].as<bool>();
  params.map_frame_id_ = adapter == "eufs" ? "base_footprint" : "map";
  params.minimum_velocity_ = planning_config["minimum_velocity"].as<double>();
  params.braking_acceleration_ = planning_config["braking_acceleration"].as<double>();
  params.normal_acceleration_ = planning_config["normal_acceleration"].as<double>();
  params.use_velocity_planning_ = planning_config["use_velocity_planning"].as<bool>();

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
    this->visualization_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("/path_planning/smoothed_path", 10);

    this->triangulations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_planning/triangulations", 10);
  }
  // Publishes path from file in Skidpad & Acceleration events
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&Planning::publish_predicitive_track_points, this));
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
      common_lib::competition_logic::Mission::AUTOCROSS;

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

  std::vector<PathPoint> triangulations_path = {};
  std::vector<PathPoint> final_path = {};

  if (this->mission == common_lib::competition_logic::Mission::SKIDPAD) {
    final_path = path_calculation_.skidpad_path(this->cone_array_, this->pose);

  } else if (this->mission == common_lib::competition_logic::Mission::AUTOCROSS ||
             this->mission == common_lib::competition_logic::Mission::EBS_TEST) {
    triangulations_path = path_calculation_.no_coloring_planning(this->cone_array_, this->pose);
    // Smooth the calculated path
    final_path = path_smoothing_.smooth_path(triangulations_path, this->pose,
                                             this->initial_car_orientation_);

    double dist_from_origin = sqrt(this->pose.position.x * this->pose.position.x +
                                   this->pose.position.y * this->pose.position.y);
    if (dist_from_origin > 75.0) {
      for (auto &point : final_path) {
        point.ideal_velocity = 0.0;
      }
    } else {
      for (auto &point : final_path) {
        point.ideal_velocity = desired_velocity_;
      }
    }
  } else {
    triangulations_path = path_calculation_.no_coloring_planning(this->cone_array_, this->pose);
    // Smooth the calculated path
    final_path = path_smoothing_.smooth_path(triangulations_path, this->pose,
                                             this->initial_car_orientation_);

    if ((this->mission == common_lib::competition_logic::Mission::AUTOCROSS &&
         this->lap_counter_ >= 1) ||
        (this->mission == common_lib::competition_logic::Mission::TRACKDRIVE &&
         this->lap_counter_ >= 10)) {
      // Correct the path orientation for Autocross
      for (auto &point : final_path) {
        point.ideal_velocity = 0.0;
      }
    } else {
      velocity_planning_.set_velocity(final_path);
    }
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

  if (planning_config_.simulation_.publishing_visualization_msgs_) {
    publish_visualization_msgs(triangulations_path, final_path);
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

void Planning::publish_visualization_msgs(const std::vector<PathPoint> &after_triangulations_path,
                                          const std::vector<PathPoint> &final_path) const {
  this->triangulations_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      after_triangulations_path, "after_triangulations_path", this->_map_frame_id_, "orange"));
  this->visualization_pub_->publish(common_lib::communication::line_marker_from_structure_array(
      final_path, "smoothed_path_planning", this->_map_frame_id_, 12, "green"));
}