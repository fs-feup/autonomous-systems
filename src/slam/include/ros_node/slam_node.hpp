#pragma once

#include <gtest/gtest_prod.h>

#include <functional>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <typeinfo>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/velocities.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_config/general_config.hpp"
#include "std_msgs/msg/float64.hpp"

/**
 * @brief Class representing the main speed_est node responsible for publishing
 * the calculated vehicle state with speed and the map. As well as, subscribing and
 * interpreting information, such as the cone's position and colors, from the
 * perception module.
 *
 */
class SLAMNode : public rclcpp::Node {
protected:
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _perception_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr _velocities_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      _vehicle_pose_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr _map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _visualization_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _correction_execution_time_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _prediction_execution_time_publisher_;
  rclcpp::TimerBase::SharedPtr _timer_; /**< timer */
  // std::shared_ptr<ExtendedKalmanFilter> _ekf_; /**< SLAM EKF object */
  std::shared_ptr<std::vector<common_lib::structures::Cone>> _perception_map_;
  std::shared_ptr<common_lib::structures::Velocities> _vehicle_state_velocities_;
  std::shared_ptr<std::vector<common_lib::structures::Cone>> _track_map_;
  std::shared_ptr<common_lib::structures::Pose> _vehicle_pose_;
  common_lib::competition_logic::Mission _mission_;
  bool _go_;  /// flag to start the mission
  bool _use_simulated_perception_;
  bool _use_simulated_velocities_;
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

  // /**
  //  * @brief Function to be called everytime information is received from the
  //  * IMU
  //  *
  //  * @param rotational_velocity
  //  * @param acceleration_x
  //  * @param acceleration_y
  //  */
  // void _imu_subscription_callback(const sensor_msgs::msg::Imu& imu_msg);

  // /**
  //  * @brief Function to be called everytime information is received from the
  //  * wheel encoders
  //  *
  //  * @param lb_speed wheel speeds in rpm
  //  * @param lf_speed wheel speeds in rpm
  //  * @param rb_speed wheel speeds in rpm
  //  * @param rf_speed wheel speeds in rpm
  //  * @param steering_angle steering angle in radians
  //  * @param timestamp timestamp of the message
  //  */
  // void _wheel_speeds_subscription_callback(double rl_speed, double rr_speed, double fl_speed,
  //                                          double fr_speed, double steering_angle,
  //                                          const rclcpp::Time& timestamp);

  // /**
  //  * @brief Executes:
  //  * - the prediction, validation and discovery steps of the EKF
  //  * - publication of localization
  //  * - publication of map
  //  *
  //  */
  // void _update_and_publish();

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

public:
  // /**
  //  * @brief Constructor of the main node, most things are received by launch parameter
  //  */
  // SLAMNode();

  /**
   * @brief Constructor that uses the parameters structure
   */
  SLAMNode(const SLAMParameters& params);
};