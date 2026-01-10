#include "planning/planning.hpp"

#include <vector>

#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"
#include "common_lib/config_load/config_load.hpp"

/*--------------------- Configuration Loading --------------------*/

PlanningParameters Planning::load_config(std::string &adapter) {
  PlanningParameters params;

  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("planning", "global", "global_config");
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Loading global config from: %s",
               global_config_path.c_str());

  YAML::Node global_config = YAML::LoadFile(global_config_path);
  adapter = global_config["global"]["adapter"].as<std::string>();
  params.simulation_using_simulated_se_ = global_config["global"]["use_simulated_se"].as<bool>();

  std::string planning_config_path =
      common_lib::config_load::get_config_yaml_path("planning", "planning", adapter);
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Loading planning config from: %s",
               planning_config_path.c_str());

  YAML::Node planning = YAML::LoadFile(planning_config_path);
  YAML::Node planning_config = planning["planning"];

  /*--------------------- Midpoint Generator Parameters --------------------*/
  params.mg_minimum_cone_distance_ = planning_config["mg_minimum_cone_distance"].as<double>();
  params.mg_maximum_cone_distance_ = planning_config["mg_maximum_cone_distance"].as<double>();
  params.mg_sliding_window_radius_ = planning_config["mg_sliding_window_radius"].as<double>();

  /*--------------------- Path Calculation Parameters --------------------*/
  params.pc_use_sliding_window_ = planning_config["pc_use_sliding_window"].as<bool>();
  params.pc_angle_gain_ = planning_config["pc_angle_gain"].as<double>();
  params.pc_distance_gain_ = planning_config["pc_distance_gain"].as<double>();
  params.pc_angle_exponent_ = planning_config["pc_angle_exponent"].as<double>();
  params.pc_distance_exponent_ = planning_config["pc_distance_exponent"].as<double>();
  params.pc_max_cost_ = planning_config["pc_max_cost"].as<double>();
  params.pc_lookback_points_ = planning_config["pc_lookback_points"].as<int>();
  params.pc_search_depth_ = planning_config["pc_search_depth"].as<int>();
  params.pc_max_points_ = planning_config["pc_max_points"].as<int>();
  params.pc_minimum_point_distance_ = planning_config["pc_minimum_point_distance"].as<double>();
  params.pc_reset_interval_ = planning_config["pc_reset_interval"].as<int>();
  params.pc_use_reset_path_ = planning_config["pc_use_reset_path"].as<bool>();

  /*--------------------- Skidpad Parameters --------------------*/
  params.skidpad_minimum_cones_ = planning_config["skidpad_minimum_cones"].as<int>();
  params.skidpad_tolerance_ = planning_config["skidpad_tolerance"].as<double>();

  /*--------------------- Path Smoothing Parameters --------------------*/
  params.smoothing_spline_precision_ = planning_config["smoothing_spline_precision"].as<int>();
  params.smoothing_spline_order_ = planning_config["smoothing_spline_order"].as<int>();
  params.smoothing_spline_coeffs_ratio_ =
      planning_config["smoothing_spline_coeffs_ratio"].as<float>();
  params.smoothing_use_path_smoothing_ = planning_config["smoothing_use_path_smoothing"].as<bool>();
  params.smoothing_use_optimization_ = planning_config["smoothing_use_optimization"].as<bool>();
  params.smoothing_car_width_ = planning_config["smoothing_car_width"].as<double>();
  params.smoothing_safety_margin_ = planning_config["smoothing_safety_margin"].as<double>();
  params.smoothing_curvature_weight_ = planning_config["smoothing_curvature_weight"].as<double>();
  params.smoothing_smoothness_weight_ = planning_config["smoothing_smoothness_weight"].as<double>();
  params.smoothing_safety_weight_ = planning_config["smoothing_safety_weight"].as<double>();
  params.smoothing_max_iterations_ = planning_config["smoothing_max_iterations"].as<int>();
  params.smoothing_tolerance_ = planning_config["smoothing_tolerance"].as<double>();

  /*--------------------- Velocity Planning Parameters --------------------*/
  params.vp_minimum_velocity_ = planning_config["vp_minimum_velocity"].as<double>();
  params.vp_braking_acceleration_ = planning_config["vp_braking_acceleration"].as<double>();
  params.vp_acceleration_ = planning_config["vp_acceleration"].as<double>();
  params.vp_normal_acceleration_ = planning_config["vp_normal_acceleration"].as<double>();
  params.vp_use_velocity_planning_ = planning_config["vp_use_velocity_planning"].as<bool>();
  params.vp_desired_velocity_ = planning_config["vp_desired_velocity"].as<double>();

  /*--------------------- Simulation Configuration Parameters --------------------*/
  params.simulation_publishing_visualization_msgs_ =
      planning_config["simulation_publishing_visualization_msgs"].as<bool>();

  if (adapter == "eufs") {
    params.map_frame_id_ = "base_footprint";
  } else {
    params.map_frame_id_ = "map";
  }

  return params;
}

/*--------------------- Constructor --------------------*/

Planning::Planning(const PlanningParameters &params)
    : Node("planning"),
      planning_config_(params),
      map_frame_id_(params.map_frame_id_),
      desired_velocity_(params.vp_desired_velocity_) {
  path_calculation_ = PathCalculation(planning_config_.path_calculation_);
  path_smoothing_ = PathSmoothing(planning_config_.smoothing_);
  velocity_planning_ = VelocityPlanning(planning_config_.velocity_planning_);
  skidpad_ = Skidpad(planning_config_.skidpad_);

  param_client_ =
      create_client<rcl_interfaces::srv::GetParameters>("/pacsim/pacsim_node/get_parameters");
  fetch_discipline();

  path_pub_ = create_publisher<custom_interfaces::msg::PathPointArray>("/path_planning/path", 10);

  planning_execution_time_pub_ =
      create_publisher<std_msgs::msg::Float64>("/path_planning/execution_time", 10);

  if (planning_config_.simulation_.publishing_visualization_msgs_) {
    yellow_cones_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/path_planning/yellow_cones", 10);
    blue_cones_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/path_planning/blue_cones", 10);
    triangulations_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("/path_planning/triangulations", 10);
    path_to_car_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/path_planning/path_to_car", 10);
    full_path_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/path_planning/full_path", 10);
    smoothed_path_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("/path_planning/smoothed_path_", 10);
    velocity_hover_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/path_planning/velocity_hover", 10);
  }

  if (!planning_config_.simulation_.using_simulated_se_) {
    vehicle_localization_sub_ = create_subscription<custom_interfaces::msg::Pose>(
        "/state_estimation/vehicle_pose", 10,
        std::bind(&Planning::vehicle_localization_callback, this, std::placeholders::_1));

    track_map_sub_ = create_subscription<custom_interfaces::msg::ConeArray>(
        "/state_estimation/map", 10,
        std::bind(&Planning::track_map_callback, this, std::placeholders::_1));

    lap_counter_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/state_estimation/lap_counter", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
          lap_counter_ = static_cast<int>(msg->data);
        });
  }

  RCLCPP_INFO(rclcpp::get_logger("planning"), "Using simulated state estimation: %d",
              planning_config_.simulation_.using_simulated_se_);
}

/*--------------------- Mission Management in Pacsim --------------------*/

void Planning::fetch_discipline() {
  Mission mission_result = Mission::NONE;

  if (!param_client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_ERROR(get_logger(), "Service /pacsim/pacsim_node/get_parameters not available.");
  } else {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("discipline");

    (void)param_client_->async_send_request(
        request, [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
          auto response = future.get();
          Mission mission_result = Mission::AUTOCROSS;

          if (!response->values.empty() && response->values[0].type == 4) {
            std::string discipline = response->values[0].string_value;
            RCLCPP_INFO(get_logger(), "Discipline received: %s", discipline.c_str());

            if (discipline == "skidpad") {
              mission_result = Mission::SKIDPAD;
            } else if (discipline == "acceleration") {
              mission_result = Mission::ACCELERATION;
            } else if (discipline == "trackdrive") {
              mission_result = Mission::TRACKDRIVE;
            } else {
              mission_result = Mission::AUTOCROSS;
            }
          } else {
            RCLCPP_ERROR(get_logger(), "Failed to retrieve discipline parameter.");
          }

          mission_ = mission_result;
        });
  }

  mission_ = mission_result;
}

void Planning::set_mission(Mission mission) { mission_ = mission; }

/*--------------------- Callbacks --------------------*/

void Planning::vehicle_localization_callback(const custom_interfaces::msg::Pose &message) {
  RCLCPP_DEBUG(get_logger(), "Received Pose: %lf - %lf - %lf", message.x, message.y, message.theta);

  pose_ = Pose(message.x, message.y, message.theta);
  path_calculation_.set_vehicle_pose(pose_);

  if (!has_received_pose_) {
    initial_car_orientation_ = message.theta;
  }

  if (has_received_track_ && !has_received_pose_) {
    run_planning_algorithms();
  }

  has_received_pose_ = true;
}

void Planning::track_map_callback(const custom_interfaces::msg::ConeArray &message) {
  int number_of_cones_received = static_cast<int>(message.cone_array.size());
  RCLCPP_DEBUG(get_logger(), "Planning received %i cones", number_of_cones_received);

  cone_array_ = common_lib::communication::cone_vector_from_custom_interfaces(message);
  has_received_track_ = true;

  if (has_received_pose_) {
    run_planning_algorithms();
  }
}

/*--------------------- Mission-Specific Planning --------------------*/

void Planning::run_ebs_test() {
  full_path_ = path_calculation_.calculate_path(cone_array_);
  smoothed_path_ = path_smoothing_.smooth_path(full_path_);

  double distance_from_origin =
      std::sqrt(pose_.position.x * pose_.position.x + pose_.position.y * pose_.position.y);

  if (distance_from_origin > 90.0) {
    if (!is_braking_) {
      is_braking_ = true;
      brake_time_ = std::chrono::steady_clock::now();
    }

    for (PathPoint &point : smoothed_path_) {
      std::chrono::duration<double> time_since_brake_start =
          std::chrono::steady_clock::now() - brake_time_;
      point.ideal_velocity =
          std::max((desired_velocity_ + (planning_config_.velocity_planning_.braking_acceleration_ *
                                         time_since_brake_start.count())),
                   0.0);
    }
  } else {
    for (PathPoint &point : smoothed_path_) {
      point.ideal_velocity = desired_velocity_;
    }
  }
}

void Planning::run_autocross() {
  if (lap_counter_ == 0) {
    full_path_ = path_calculation_.calculate_path(cone_array_);
    smoothed_path_ = path_smoothing_.smooth_path(full_path_);
    velocity_planning_.set_velocity(smoothed_path_);
  }
  if (lap_counter_ >= 1) {
    if (!is_map_closed_) {
      is_map_closed_ = true;
      full_path_ = path_calculation_.calculate_trackdrive(cone_array_);
      smoothed_path_ = path_smoothing_.smooth_path(full_path_);
      velocity_planning_.set_velocity(smoothed_path_);
    }
    velocity_planning_.stop(smoothed_path_);
  }
}

void Planning::run_trackdrive() {
  if (lap_counter_ == 0) {
    full_path_ = path_calculation_.calculate_path(cone_array_);
    smoothed_path_ = path_smoothing_.smooth_path(full_path_);
    velocity_planning_.set_velocity(smoothed_path_);
  } else if (lap_counter_ >= 1 && lap_counter_ < 10) {
    if (!is_map_closed_) {
      is_map_closed_ = true;
      full_path_ = path_calculation_.calculate_trackdrive(cone_array_);

      const std::vector<Cone> yellow_cones_ = path_calculation_.get_yellow_cones();
      const std::vector<Cone> blue_cones_ = path_calculation_.get_blue_cones();
      std::vector<PathPoint> yellow_cones;
      std::vector<PathPoint> blue_cones;
      for (const Cone &cone : yellow_cones_) {
        (void)yellow_cones.emplace_back(cone.position.x, cone.position.y);
      }
      for (const Cone &cone : blue_cones_) {
        (void)blue_cones.emplace_back(cone.position.x, cone.position.y);
      }

      smoothed_path_ = path_smoothing_.optimize_path(full_path_, yellow_cones, blue_cones);
      velocity_planning_.trackdrive_velocity(smoothed_path_);

      if (!smoothed_path_.empty()) {
        double sum = 0.0;
        double max_vel = smoothed_path_[0].ideal_velocity;
        double min_vel = smoothed_path_[0].ideal_velocity;

        for (const auto &point : smoothed_path_) {
          sum += point.ideal_velocity;
          if (point.ideal_velocity > max_vel) {
            max_vel = point.ideal_velocity;
          }
          if (point.ideal_velocity < min_vel) {
            min_vel = point.ideal_velocity;
          }
        }

        double avg_vel = sum / smoothed_path_.size();

        RCLCPP_INFO(get_logger(),
                    "[TRACKDRIVE] Velocity Stats - Avg: %.2f m/s | Max: %.2f m/s | Min: %.2f m/s",
                    avg_vel, max_vel, min_vel);

        RCLCPP_INFO(get_logger(), "[TRACKDRIVE] Path velocities (first 20 & last 20 points):");

        size_t n = smoothed_path_.size();
        size_t count = 20;

        // First 20 points
        for (size_t i = 0; i < std::min(n, count); ++i) {
          const auto &p = smoothed_path_[i];
          RCLCPP_INFO(get_logger(), "  %zu : %.3f", i, p.ideal_velocity);
        }
        if (n > 2 * count) {
          RCLCPP_INFO(get_logger(), "  ...");
        }
        for (size_t i = (n > count ? n - count : 0); i < n; ++i) {
          const auto &p = smoothed_path_[i];
          RCLCPP_INFO(get_logger(), "  %zu : %.3f", i, p.ideal_velocity);
        }
      }
    }
  } else {
    velocity_planning_.stop(smoothed_path_);
  }
}

/*--------------------- Planning Algorithm Execution --------------------*/

void Planning::run_planning_algorithms() {
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Running Planning Algorithms");

  if (cone_array_.empty()) {
    publish_path_points();
    return;
  }

  rclcpp::Time start_time = now();

  switch (mission_) {
    case Mission::NONE:
      RCLCPP_ERROR(get_logger(), "Mission is NONE, cannot run planning algorithms.");
      return;

    case Mission::SKIDPAD:
      smoothed_path_ = skidpad_.skidpad_path(cone_array_, pose_);
      break;

    case Mission::ACCELERATION:
    case Mission::EBS_TEST:
      run_ebs_test();
      break;

    case Mission::AUTOCROSS:
      run_trackdrive();
      break;

    case Mission::TRACKDRIVE:
      run_trackdrive();
      break;

    default:
      full_path_ = path_calculation_.calculate_path(cone_array_);
      smoothed_path_ = path_smoothing_.smooth_path(full_path_);
      velocity_planning_.set_velocity(smoothed_path_);
      break;
  }

  if (smoothed_path_.size() < 10) {
    RCLCPP_INFO(rclcpp::get_logger("planning"), "Final path size: %d",
                static_cast<int>(smoothed_path_.size()));
  }

  publish_execution_time(start_time);
  publish_path_points();

  RCLCPP_DEBUG(get_logger(), "Planning will publish %i path points\n",
               static_cast<int>(smoothed_path_.size()));

  if (planning_config_.simulation_.publishing_visualization_msgs_) {
    publish_visualization_msgs();
  }
}

/*--------------------- Publishing --------------------*/

void Planning::publish_path_points() const {
  custom_interfaces::msg::PathPointArray message =
      common_lib::communication::custom_interfaces_array_from_vector(smoothed_path_,
                                                                     is_map_closed_);
  path_pub_->publish(message);
}

void Planning::publish_execution_time(rclcpp::Time start_time) {
  rclcpp::Time end_time = now();
  std_msgs::msg::Float64 planning_execution_time;
  planning_execution_time.data = (end_time - start_time).seconds() * 1000;
  planning_execution_time_pub_->publish(planning_execution_time);
}

void Planning::publish_visualization_msgs() const {
  yellow_cones_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      path_calculation_.get_yellow_cones(), "map_cones", "map", "yellow"));

  blue_cones_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      path_calculation_.get_blue_cones(), "map_cones", "map", "blue"));

  triangulations_pub_->publish(common_lib::communication::lines_marker_from_triangulations(
      path_calculation_.get_triangulations(), "triangulations", map_frame_id_, 20, "white", 0.05f,
      visualization_msgs::msg::Marker::MODIFY));

  full_path_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      full_path_, "full_path", map_frame_id_, "orange"));

  smoothed_path_pub_->publish(common_lib::communication::line_marker_from_structure_array(
      smoothed_path_, "smoothed_path__planning", map_frame_id_, 12, "green"));

  path_to_car_pub_->publish(common_lib::communication::marker_array_from_structure_array(
      path_calculation_.get_path_to_car(), "global_path", map_frame_id_, "white", "cylinder", 0.6,
      visualization_msgs::msg::Marker::MODIFY));

  if (planning_config_.smoothing_.use_path_smoothing_) {
    velocity_hover_pub_->publish(common_lib::communication::velocity_hover_markers(
        smoothed_path_, "velocity", map_frame_id_, 0.25f,
        planning_config_.smoothing_.spline_precision_));
  } else {
    velocity_hover_pub_->publish(common_lib::communication::velocity_hover_markers(
        smoothed_path_, "velocity", map_frame_id_, 0.25f, 1));
  }
}
