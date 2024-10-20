#pragma once

#include <functional>
#include <map>
#include <memory>
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
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "planning/cone_coloring.hpp"
#include "planning/outliers.hpp"
#include "planning/path_calculation.hpp"
#include "planning/smoothing.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "utils/files.hpp"

using PathPoint = common_lib::structures::PathPoint;
using Pose = common_lib::structures::Pose;

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
  common_lib::competition_logic::Mission mission =
      common_lib::competition_logic::Mission::NONE; /**< Current planning mission */

  PlanningConfig planning_config_;

  ConeColoring cone_coloring_;
  Outliers outliers_;
  PathCalculation path_calculation_;
  PathSmoothing path_smoothing_;
  double desired_velocity_;
  double initial_car_orientation_;

  std::map<common_lib::competition_logic::Mission, std::string> predictive_paths_ = {
      {common_lib::competition_logic::Mission::ACCELERATION, "/events/acceleration.txt"},
      {common_lib::competition_logic::Mission::SKIDPAD,
       "/events/skidpad.txt"}}; /**< Predictive paths for different missions */

  std::string _map_frame_id_; /**< Frame ID for the map */
  bool received_first_track_ = false;
  bool received_first_pose_ = false;
  std::vector<Cone> cone_array_;
  /**< Subscription to vehicle localization */
  rclcpp::Subscription<custom_interfaces::msg::VehicleState>::SharedPtr vl_sub_;
  /**< Subscription to track map */
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
  /**< Local path points publisher */
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr local_pub_;
  /**< Publisher for the final path*/
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization_pub_;
  /**< Publisher for blue cones after cone coloring*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr blue_cones_pub_;
  /**< Publisher for yellow cones after cone coloring*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr yellow_cones_pub_;
  /**< Publisher for path after triangulations */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr triangulations_pub_;
  /**< Timer for the periodic publishing */
  /**< Publisher for blue cones after cone coloring*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr after_rem_blue_cones_pub_;
  /**< Publisher for yellow cones after cone coloring*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr after_rem_yellow_cones_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _planning_execution_time_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief Callback for vehicle localization updates (undefined).
   *
   * @param msg The received VehicleState message.
   */
  void vehicle_localization_callback(const custom_interfaces::msg::VehicleState &msg);
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
   * @brief Publishes predictive track points.
   * @details Depending on the selected mission, this function publishes
   * predicted path points (published the path for the missions we previously
   * know the track for).
   */
  void publish_predicitive_track_points();

  /**
   * @brief publish all visualization messages from the planning node
   *
   * @param current_left_cones left cones after cone coloring
   * @param current_right_cones right cones after cone coloring
   * @param after_triangulations_path path after triangulations
   * @param final_path final path after smoothing
   */
  void publish_visualization_msgs(const std::vector<Cone> &left_cones,
                                  const std::vector<Cone> &right_cones,
                                  const std::vector<Cone> &after_refining_blue_cones,
                                  const std::vector<Cone> &after_refining_yellow_cones,
                                  const std::vector<PathPoint> &after_triangulations_path,
                                  const std::vector<PathPoint> &final_path) const;

  /**
   * @brief Checks if the current mission is predictive.
   *
   * @return True if the mission is predictive, false otherwise.
   */
  bool is_predicitve_mission() const;

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
  /**
   * @brief Set the mission for planning.
   *
   * @param mission The mission to set for planning (e.g.,
   * Mission::acceleration, Mission::skidpad).
   * @details This method configures the Planning node for a specific mission
   * type, possibly affecting its behavior if used.
   */
  void set_mission(common_lib::competition_logic::Mission mission);

  friend class PacSimAdapter;

  friend class EufsAdapter;

  friend class FsdsAdapter;

  friend class VehicleAdapter;
};
