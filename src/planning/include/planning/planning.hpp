#pragma once

#include <yaml-cpp/yaml.h>

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
#include "planning/velocity_planning.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "utils/files.hpp"

using PathPoint = common_lib::structures::PathPoint;
using Pose = common_lib::structures::Pose;
using Mission = common_lib::competition_logic::Mission;

using std::placeholders::_1;

/**
 * @class Planning
 * @brief Class responsible for planning and path generation for our car.
 *
 * This class inherits from rclcpp::Node and provides functionalities for
 * handling different missions, subscribing to vehicle localization and track
 * map topics, and publishing planned path points.
 */
class Planning : public rclcpp::Node {
private: 
  Mission mission_ = Mission::NONE; /**< Current planning mission */

  PlanningConfig planning_config_;

  Outliers outliers_;
  PathCalculation path_calculation_;
  PathSmoothing path_smoothing_;
  VelocityPlanning velocity_planning_;
  double desired_velocity_;
  double initial_car_orientation_;
  int lap_counter_ = 0;

  bool is_braking_ = false; /**< Flag to indicate if it is braking */
  std::chrono::steady_clock::time_point brake_time_;

  /**< Vector of path points representing the complete planned path from start to finish. */
  std::vector<PathPoint> full_path_ = {};
  /**< Vector of path points representing the final smoothed path used for planning. */
  std::vector<PathPoint> final_path_ = {}; 
  /**< Vector of path points representing the past portion of the path 
   *  (from the start to a lookback distance behind the car’s current position) */
  std::vector<PathPoint> past_path_ = {};

  // For Trackdrive
  bool has_found_full_path_ = false;      // for Trackdrive

  //bool path_orientation_corrected_ = false;                                     // for Skidpad
  std::vector<PathPoint> predefined_path_;                                      // for Skidpad
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client_;  // for mission logic

  std::string map_frame_id_; /**< Frame ID for the map */
  bool has_received_track_ = false;
  bool has_received_pose_ = false;
  std::vector<Cone> cone_array_;
  /**< Subscription to vehicle localization */
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vl_sub_;
  /**< Subscription to track map */
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
  /**< Local path points publisher */
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr local_pub_;
  
  //Visualization publishers:
  // /**< Publisher for Delaunay triangulations */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr triangulations_pub_;
  /**< Publisher for the past portion of the path 
    (from the start to a lookback distance behind the car’s current position) */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr past_path_pub_;
  /**< Publisher for the entire planned path (from start to finish)*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr full_path_pub_;
  /**< Publisher for the smoothed path*/
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr final_path_pub_;

  
  /**< Timer for the periodic publishing */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _planning_execution_time_publisher_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _lap_counter_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mission_finished_client_;

  /**
   * @brief Callback for vehicle localization updates (undefined).
   *
   * @param msg The received Pose message.
   */
  void vehicle_localization_callback(const custom_interfaces::msg::Pose &msg);

  /**
   * @brief Fetches the mission from pacsim and updates the mission_ member variable.
   * 
   * Defaults to Mission::AUTOCROSS if parameter retrieval fails.
   */
  void fetch_discipline();

  /**
   * @brief Callback for track map updates(when msg received).
   * 
   * Processes incoming cone map and calls run_planning_algorithm if vehicle pose
   * has already been received.
   * 
   * @param msg The received ConeArray message.
   */
  void track_map_callback(const custom_interfaces::msg::ConeArray &msg);

  /**
   * @brief Runs the planning algorithms. Called from the callbacks
   * 
   * Dispatches to appropriate mission-specific planning method based on current 
   * mission type. Handles empty cone arrays, publishes execution time, track points,
   * and visualization messages
   */
  void run_planning_algorithms();

  /**
   * @brief Publishes a list of path points to Control.
   *
   * Converts final_path_ member to an PathPointArray and publishes
   * to /path_planning/path topic for control.
   */
  void publish_track_points() const;

  /**
   * @brief publish all visualization messages from the planning node
   * 
   */
  void publish_visualization_msgs() const;
  
  /**
   * @brief Executes planning for EBS (Emergency Brake System) test mission
   * 
   * Calculates the path, smooths it, and implements distance-based
   * braking logic. Starts braking when vehicle is more than 90m from origin,
   * applying deceleration until vehicle stops.
   */
  void run_ebs_test();

  /**
   * @brief Executes planning for trackdrive mission
   * 
   * Implements multi-lap logic:
   * - Lap 0: Explores track and builds path
   * - Laps 1-9: Uses optimized full track path with velocity planning
   * - Lap 10+: Brings vehicle to stop
   */
  void run_trackdrive();

  /**
   * @brief Calculates and publishes planning algorithm execution time
   * 
   * Computes time elapsed since start_time and publishes result in milliseconds
   * to /path_planning/execution_time topic.
   * 
   * @param start_time ROS time when planning algorithms started execution
   */
  void publish_execution_time(rclcpp::Time start_time);

  /**
   * @brief Executes planning for autocross mission
   * 
   * Calculates the path, smooths it, applies the velocity planning,
   * and stops vehicle after completing one lap.
   */
  void run_autocross();


  virtual void finish() = 0;

  /**
   * @brief current vehicle pose
   */
  Pose pose_;

public:
  /**
   * @brief Constructor for the Planning class.
   *
   * Initializes an instance of the Planning class (a ROS node) with essential
   * communication components.
   *
   * This constructor sets up subscriptions to vehicle localization and mapping
   * topics, and creates publishers for local and (global X) path planning
   * results. It also establishes a timer for periodic tasks related to (local)
   * publishing info to topics. Additionally, it initializes an Adapter instance
   * for communication with external systems.
   */
  explicit Planning(const PlanningParameters &params);

  /**
   * @brief Loads planning configuration parameters from YAML files
   * 
   * @param adapter Reference to string that will store the adapter type ("eufs", "pacsim", "vehicle")
   * @return PlanningParameters Struct containing all loaded configuration parameters
   */
  static PlanningParameters load_config(std::string &adapter);

  /**
   * @brief Set the mission for planning.
   *
   * @param mission The mission to set for planning (e.g.,
   * Mission::acceleration, Mission::skidpad).
   * 
   * @details This method configures the Planning node for a specific mission
   * type, possibly affecting its behavior if used.
   */
  void set_mission(Mission mission);

  friend class PacSimAdapter;

  friend class EufsAdapter;

  friend class FsdsAdapter;

  friend class VehicleAdapter;
};
