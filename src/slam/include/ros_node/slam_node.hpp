#pragma once

#include <gtest/gtest_prod.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/velocities.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "perception_sensor_lib/data_association/maximum_likelihood_md.hpp"
#include "perception_sensor_lib/data_association/maximum_likelihood_nll.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_config/general_config.hpp"
#include "slam_solver/slam_solver.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

/**
 * @brief Class representing the main SLAM node responsible for publishing
 * the calculated pose and map.
 *
 */
class SLAMNode : public rclcpp::Node {
protected:
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _perception_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr _velocities_subscription_;
  rclcpp::Publisher<custom_interfaces::msg::Pose>::SharedPtr _vehicle_pose_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr _map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _visualization_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      _visualization_perception_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _execution_time_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _covariance_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _lap_counter_publisher_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr _timer_;      /**< timer */
  std::shared_ptr<SLAMSolver> _slam_solver_; /**< SLAM solver object */
  std::vector<common_lib::structures::Cone> _perception_map_;
  common_lib::structures::Velocities _vehicle_state_velocities_;
  std::vector<common_lib::structures::Cone> _track_map_;
  common_lib::structures::Pose _vehicle_pose_;
  std::shared_ptr<std::vector<double>>
      _execution_times_;  //< Execution times: 0 -> total motion; 1
                          //-> total observation; the rest are solver specific
  common_lib::competition_logic::Mission _mission_ = common_lib::competition_logic::Mission::NONE;
  bool _go_;  /// flag to start the mission
  std::string _adapter_name_;

  /**
   * @brief Callback that updates everytime information
   * is received from the perception module
   *
   * @param msg Message containing the array of perceived cones
   */
  void _perception_subscription_callback(const custom_interfaces::msg::ConeArray& msg);

  /**
   * @brief Callback that updates everytime information
   * is received from vehicle state estimation node
   *
   * @param msg Message containing the velocitites of the vehicle
   */
  void _velocities_subscription_callback(const custom_interfaces::msg::Velocities& msg);

  /**
   * @brief publishes the localization ('vehicle_pose') to the topic
   * vehicle_pose
   *
   */
  void _publish_vehicle_pose();

  /**
   * @brief publishes the map ('track_map') to the topic track_map
   *
   */
  void _publish_map();

  /**
   * @brief publishes the covariance of the state
   *
   */
  void _publish_covariance();

  /**
   * @brief publishes the lap counter
   *
   */
  void _publish_lap_counter();

public:
  // /**
  //  * @brief Constructor of the main node, most things are received by launch parameter
  //  */
  // SLAMNode();

  /**
   * @brief Constructor that uses the parameters structure
   */
  SLAMNode(const SLAMParameters& params);

  /**
   * @brief Initialize functions
   * @description This method is used to initialize things that require the constructed node
   */
  void init();
};