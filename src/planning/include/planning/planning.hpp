#ifndef SRC_PLANNING_INCLUDE_PLANNING_PLANNING_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PLANNING_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

// #include "adapter/adapter.hpp"

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "utils/files.hpp"
#include "utils/mission.hpp"

using std::placeholders::_1;

class Adapter;

/**
 * @class Planning
 * @brief Class responsible for planning and path generation for our car.
 *
 * This class inherits from rclcpp::Node and provides functionalities for
 * handling different missions, subscribing to vehicle localization and track
 * map topics, and publishing planned path points.
 */
class Planning : public rclcpp::Node {
  Mission mission = not_selected;                                /**< Current planning mission */
  LocalPathPlanner *local_path_planner = new LocalPathPlanner(); /**< Local path planner instance */
  Adapter *adapter;           /**< Adapter instance for external communication */
  std::string mode = "fsds";  // Temporary, change as desired. TODO(andre): Make not hardcoded

  std::map<Mission, std::string> predictive_paths = {
      {Mission::acceleration, "/events/acceleration.txt"},
      {Mission::skidpad, "/events/skidpad.txt"}}; /**< Predictive paths for different missions */

  /**< Subscription to vehicle localization */
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vl_sub_;
  /**< Subscription to track map */
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
  /**< Local path points publisher */
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr local_pub_;
  /**< Global path points publisher */
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr global_pub_;
  /**< Timer for the periodic publishing */
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief Callback for vehicle localization updates (undefined).
   *
   * @param msg The received Pose message.
   */
  void vehicle_localization_callback(const custom_interfaces::msg::Pose msg);
  /**
   * @brief Callback for track map updates(when msg received).
   *
   * @param msg The received ConeArray message.
   * @details creates track with the received cones(position), then calculates
   * path by calling method from the local path planner and finnaly publishes
   * the path by calling the publish method
   */
  void track_map_callback(const custom_interfaces::msg::ConeArray msg);
  /**
   * @brief Publishes a list of path points.
   *
   * @param path A vector of Position pointers representing the path points.
   * @details This function publishes the provided path points as a custom
   * created PointArray message.
   */

  void publish_track_points(std::vector<PathPoint *> path) const;
  /**
   * @brief Publishes predictive track points.
   * @details Depending on the selected mission, this function publishes
   * predicted path points (published the path for the missions we previously
   * know the track for).
   */
  void publish_predicitive_track_points();
  /**
   * @brief Checks if the current mission is predictive.
   *
   * @return True if the mission is predictive, false otherwise.
   */
  bool is_predicitve_mission() const;

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
  Planning();
  /**
   * @brief Set the mission for planning.
   *
   * @param mission The mission to set for planning (e.g.,
   * Mission::acceleration, Mission::skidpad).
   * @details This method configures the Planning node for a specific mission
   * type, possibly affecting its behavior if used.
   */
  void set_mission(Mission mission);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PLANNING_HPP_