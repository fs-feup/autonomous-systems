#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "common_lib/communication/interfaces.hpp"
#include "common_lib/communication/marker.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "config/planning_config.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "planning/outliers.hpp"
#include "planning/path_calculation.hpp"
#include "planning/smoothing.hpp"
#include "planning/skidpad.hpp"
#include "planning/velocity_planning.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

using PathPoint = common_lib::structures::PathPoint;
using Pose = common_lib::structures::Pose;
using Mission = common_lib::competition_logic::Mission;


/**
 * @class Planning
 * @brief Responsible for path planning and trajectory generation for the autonomous vehicle.
 *
 * The Planning class handles multiple mission types (acceleration, skidpad, autocross, trackdrive)
 * and generates optimal paths based on cone positions. It subscribes to vehicle localization and
 * track map data, performs path calculation and smoothing, applies velocity planning, and publishes
 * the final trajectory for the control module.
 *
 * This class integrates several planning modules:
 * - Outlier removal for noisy cone data
 * - Path calculation using computational geometry
 * - Path smoothing using spline interpolation
 * - Velocity planning with acceleration constraints
 * - Skidpad-specific planning
 *
 * @note This class inherits from rclcpp::Node and operates as a ROS 2 node.
 */
class Planning : public rclcpp::Node {
public:
  /**
   * @brief Constructs a Planning node with the specified configuration parameters.
   *
   * Initializes all planning modules, subscriptions, and publishers. Sets up communication
   * with state estimation and pacsim services. Determines the operating adapter type
   * and configures frame IDs accordingly.
   *
   * @param params Configuration parameters loaded from YAML files
   */
  explicit Planning(const PlanningParameters &params);

  /**
   * @brief Loads planning configuration from YAML files.
   *
   * Reads global configuration to determine the adapter type, then loads adapter-specific
   * planning parameters from the corresponding YAML file.
   *
   * @param adapter Output parameter that stores the adapter type ("eufs", "pacsim", "vehicle")
   * @return PlanningParameters Struct containing all loaded configuration parameters
   */
  static PlanningParameters load_config(std::string &adapter);

  /**
   * @brief Sets the mission type for planning execution.
   *
   * @param mission The mission type to execute
   */
  void set_mission(Mission mission);

  friend class PacSimAdapter;
  friend class EufsAdapter;
  friend class FsdsAdapter;
  friend class VehicleAdapter;

private:
  /*--------------------- Mission and Configuration --------------------*/
  Mission mission_ = Mission::NONE;
  PlanningConfig planning_config_;
  Pose pose_;
  std::string map_frame_id_;
  double desired_velocity_;
  double initial_car_orientation_;
  int lap_counter_ = 0;

  /*--------------------- Planning Modules --------------------*/
  Outliers outliers_;
  PathCalculation path_calculation_;
  PathSmoothing path_smoothing_;
  VelocityPlanning velocity_planning_;
  Skidpad skidpad_;

  /*--------------------- State Tracking --------------------*/
  bool is_braking_ = false;
  bool has_received_track_ = false;
  bool has_received_pose_ = false;
  bool has_found_full_path_ = false;
  std::chrono::steady_clock::time_point brake_time_;

  /*--------------------- Path Data --------------------*/
  
  std::vector<PathPoint> full_path_;
  std::vector<PathPoint> final_path_;
  std::vector<Cone> cone_array_;

  /*--------------------- Subscriptions --------------------*/
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vehicle_localization_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lap_counter_sub_;

  /*--------------------- Publishers --------------------*/
  /**< Publisher of the smoothed path to control */
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr planning_execution_time_pub_;

  /*--------------------- Visualization Publishers --------------------*/
  /**< Publisher for Delaunay triangulations */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr triangulations_pub_;
  /**< Publisher for the past portion of the path 
    (from the start to a lookback distance behind the car’s current position) */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_to_car_pub_;
  /**< Publisher for the entire planned path (from start to finish)*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr full_path_pub_;
  /**< Publisher for the smoothed path*/
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr final_path_pub_;

  /*--------------------- Service Clients --------------------*/
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client_;

  /*--------------------- Callbacks --------------------*/
  /**
   * @brief Callback for vehicle localization pose updates.
   *
   * Updates the current vehicle pose and triggers run_planning_algorithms() if
   * track map have been received.
   *
   * @param message The received Pose message containing vehicle position and orientation
   */
  void vehicle_localization_callback(const custom_interfaces::msg::Pose &message);

  /**
   * @brief Callback for track map updates.
   *
   * Processes incoming cone detections and triggers run_planning_algorithms() if vehicle pose
   * has already been received.
   *
   * @param message The received ConeArray message
   */
  void track_map_callback(const custom_interfaces::msg::ConeArray &message);

  /*--------------------- Mission-Specific Planning --------------------*/
  /**
   * @brief Executes planning for the EBS (Emergency Braking System) test mission.
   *
   * Calculates and smooths the path, then implements distance-based braking logic.
   * Braking is triggered when the vehicle is more than 90m from origin, applying
   * deceleration until the vehicle stops.
   */
  void run_ebs_test();

  /**
   * @brief Executes planning for the autocross mission.
   *
   * On the first lap, calculates and smooths the path with velocity planning.
   * On subsequent laps, reuses the calculated full path and applies stopping logic
   * after the first lap completion.
   */
  void run_autocross();

  /**
   * @brief Executes planning for the trackdrive mission.
   *
   * Implements multi-lap logic:
   * - Lap 0: Explores track and builds initial path
   * - Laps 1-9: Uses optimized full track path with trackdrive velocity planning
   * - Lap 10+: Brings vehicle to stop using full path
   */
  void run_trackdrive();

  /*--------------------- Core Planning Operations --------------------*/
  /**
   * @brief Fetches the current mission/discipline from the pacsim simulation service.
   *
   */
  void fetch_discipline();

  /**
   * @brief Calculates path and applies smoothing.
   *
   * Performs path calculation based on cone positions and then smooths the path.
   */
  void calculate_and_smooth_path();

  /**
   * @brief Runs the appropriate planning algorithm based on current mission.
   *
   * Dispatches to mission-specific planning functions and publishes the resulting
   * trajectory.
   */
  void run_planning_algorithms();

  /*--------------------- Publishing --------------------*/
  /**
   * @brief Publishes the final planned path to the control module.
   *
   */
  void publish_path_points() const;

  /**
   * @brief Publishes all visualization markers.
   *
   * Publishes triangulation, full path, smoothed path, and path_to_car markers.
   * 
   */
  void publish_visualization_msgs() const;

  /**
   * @brief Publishes the planning algorithm execution time.
   *
   *
   * @param start_time ROS time when planning algorithms began execution
   */
  void publish_execution_time(rclcpp::Time start_time);

  /*--------------------- Abstract Methods --------------------*/
  /**
   * @brief Called when planning mission is completed.
   *
   */
  virtual void finish() = 0;
};