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
  Mission mission = Mission::NONE; /**< Current planning mission */

  PlanningConfig planning_config_;

  Outliers outliers_;
  PathCalculation path_calculation_;
  PathSmoothing path_smoothing_;
  VelocityPlanning velocity_planning_;
  double desired_velocity_;
  double initial_car_orientation_;
  int lap_counter_ = 0;

  bool braking_ = false; /**< Flag to indicate if it is braking */
  std::chrono::steady_clock::time_point brake_time_;

  // For Trackdrive
  std::vector<PathPoint> full_path_;  // for Trackdrive
  bool found_full_path_ = false;      // for Trackdrive

  bool path_orientation_corrected_ = false;                                     // for Skidpad
  std::vector<PathPoint> predefined_path_;                                      // for Skidpad
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client_;  // for mission logic

  std::string _map_frame_id_; /**< Frame ID for the map */
  bool received_first_track_ = false;
  bool received_first_pose_ = false;
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
   * @brief Fetches the mission from the parameters.
   */
  void fetch_discipline();

  /**
   * @brief Callback for track map updates(when msg received).
   *
   * @param msg The received ConeArray message.
   * @details creates track with the received cones(position), then calculates
   * path by calling method from the local path planner and finnaly publishes
   * the path by calling the publish method
   */
  void track_map_callback(const custom_interfaces::msg::ConeArray &msg);

  /**
   * @brief Runs the planning algorithms. Called from the callbacks
   * @details This function creates a Track instance, a ConeColoring instance, a
   * PathSmoothing instance, and runs the planning algorithms to generate a path.
   */
  void run_planning_algorithms();
  /**
   * @brief Publishes a list of path points.
   *
   * @param path A vector of Position pointers representing the path points.
   * @details This function publishes the provided path points as a custom
   * created PointArray message.
   */

  void publish_track_points(const std::vector<PathPoint> &path) const;

  /**
   * @brief publish all visualization messages from the planning node
   * 
   * @param full_path   Vector of path points representing the complete planned path from start to finish..
   * @param final_path  Vector of path points representing the final smoothed path used for planning.
   * @param past_path Vector of path points representing the past portion of the path 
   *  (from the start to a lookback distance behind the car’s current position)
   */
  void publish_visualization_msgs(const std::vector<PathPoint> &full_path,
                                  const std::vector<PathPoint> &final_path,
                                  const std::vector<PathPoint> &past_path) const;
  
  
  //hhhh
  void ebs_test(std::vector<PathPoint>& full_path, std::vector<PathPoint>& final_path);
  //teste
  void trackdrive(std::vector<PathPoint>& full_path, std::vector<PathPoint>& final_path, std::vector<PathPoint>& past_path);
  //same
  void publish_execution_time(rclcpp::Time start_time);
  //same
  void autocross(std::vector<PathPoint>& full_path, std::vector<PathPoint>& final_path, std::vector<PathPoint>& past_path);


  virtual void finish() = 0;

  /**
   * @brief current vehicle pose
   *
   */
  Pose pose;

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

  static PlanningParameters load_config(std::string &adapter);
  /**
   * @brief Set the mission for planning.
   *
   * @param mission The mission to set for planning (e.g.,
   * Mission::acceleration, Mission::skidpad).
   * @details This method configures the Planning node for a specific mission
   * type, possibly affecting its behavior if used.
   */
  void set_mission(Mission mission);

  friend class PacSimAdapter;

  friend class EufsAdapter;

  friend class FsdsAdapter;

  friend class VehicleAdapter;
};
