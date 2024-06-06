#pragma once

#include <gtest/gtest_prod.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <typeinfo>


#include "common_lib/competition_logic/mission_logic.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "kalman_filter/ekf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/**
 * @brief Class representing the main speed_est node responsible for publishing
 * the calculated vehicle state with speed and the map. As well as, subscribing and
 * interpreting information, such as the cone's position and colors, from the
 * perception module.
 *
 */
class SENode : public rclcpp::Node {
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _perception_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription_;
  rclcpp::Publisher<custom_interfaces::msg::VehicleState>::SharedPtr _vehicle_state_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr _map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _visualization_map_publisher_;
  rclcpp::TimerBase::SharedPtr _timer_;        /**< timer */
  std::shared_ptr<ExtendedKalmanFilter> _ekf_; /**< SLAM EKF object */
  std::shared_ptr<std::vector<common_lib::structures::Cone>> _perception_map_;
  std::shared_ptr<MotionUpdate> _motion_update_;
  std::shared_ptr<std::vector<common_lib::structures::Cone>> _track_map_;
  std::shared_ptr<common_lib::structures::VehicleState> _vehicle_state_;
  common_lib::competition_logic::Mission _mission_;
  bool _go_;  /// flag to start the mission
  bool _use_odometry_;
  bool _use_simulated_perception_;
  std::mutex _mutex_;  /// Mutex used to lock EKF access

  /**
   * @brief Callback that updates everytime information
   * is received from the perception module
   *
   * @param msg Message containing the array of perceived cones
   */
  void _perception_subscription_callback(const custom_interfaces::msg::ConeArray& msg);

  /**
   * @brief Function to be called everytime information is received from the
   * IMU
   *
   * @param rotational_velocity
   * @param acceleration_x
   * @param acceleration_y
   */
  void imu_subscription_callback(const sensor_msgs::msg::Imu& imu_msg);

  /**
   * @brief Function to be called everytime information is received from the
   * wheel encoders
   *
   * @param lb_speed wheel speeds in rpm
   * @param lf_speed wheel speeds in rpm
   * @param rb_speed wheel speeds in rpm
   * @param rf_speed wheel speeds in rpm
   * @param steering_angle steering angle in radians
   * @param timestamp timestamp of the message
   */
  void _wheel_speeds_subscription_callback(double lb_speed, double lf_speed, double rb_speed,
                                           double rf_speed, double steering_angle,
                                           const rclcpp::Time& timestamp);

  /**
   * @brief Executes:
   * - the prediction, validation and discovery steps of the EKF
   * - publication of localization
   * - publication of map
   *
   */
  void _update_and_publish();

  /**
   * @brief publishes the localization ('vehicle_localization') to the topic
   * vehicle_location
   *
   */
  void _publish_vehicle_state();

  /**
   * @brief publishes the map ('track_map') to the topic track_map
   *
   */
  void _publish_map();

  /**
   * @brief executes the prediction step of the EKF
   *
   */
  void _ekf_step();

  /**
   * @brief Converts the odometry data to translational and rotational
   * velocities
   *
   * @param lb_speed wheel speeds in rpm
   * @param lf_speed wheel speeds in rpm
   * @param rb_speed wheel speeds in rpm
   * @param rf_speed wheel speeds in rpm
   * @param steering_angle steering angle in radians
   * @return MotionUpdate transformed motion update data
   */
  static MotionUpdate odometry_to_velocities_transform(double lb_speed,
                                                       [[maybe_unused]] double lf_speed,
                                                       double rb_speed,
                                                       [[maybe_unused]] double rf_speed,
                                                       double steering_angle);

  virtual void finish() = 0;  ///< Function that sends the finish signal to the respective node

public:
  /**
   * @brief Constructor of the main node, most things are received by launch parameter
   */
  SENode(bool use_odometry, bool use_simulated_perception, std::string motion_model_name,
         std::string data_assocation_model_name, float sml_da_curvature, float sml_initial_limit,
         float observation_noise, float wheel_speed_sensor_noise,
         float data_association_limit_distance);

  friend class EufsAdapter;
  friend class FsdsAdapter;
  friend class PacsimAdapter;
  friend class VehicleAdapter;
};