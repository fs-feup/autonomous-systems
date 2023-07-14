#ifndef SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_
#define SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_

#include <gtest/gtest_prod.h>

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "kalman_filter/ekf.hpp"
#include "loc_map/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Class for the node responsible for subscribing the cones' colors and
 * coordinates published by the perception module.
 */
class LMNode : public rclcpp::Node {
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _perception_subscription;
  rclcpp::Publisher<custom_interfaces::msg::Pose>::SharedPtr _localization_publisher;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr _mapping_publisher;
  rclcpp::TimerBase::SharedPtr _timer; /**< timer */
  ExtendedKalmanFilter* _ekf;          /**< SLAM EKF object */
  Map* _perception_map;
  MotionUpdate* _motion_update;
  Map* _track_map;
  VehicleState* _vehicle_state;
  Mission _mission;
  bool _use_odometry;

  /**
   * @brief function to be called everytime information is received from the
   * perception module
   */
  void _perception_subscription_callback(const custom_interfaces::msg::ConeArray message);

  /**
   * @brief function to be called everytime information is received from the
   * imu
   *
   * @param rotational_velocity
   * @param acceleration_x
   * @param acceleration_y
   */
  void _imu_subscription_callback(double rotational_velocity, double acceleration_x,
                                  double acceleration_y);

  /**
   * @brief function to be called everytime information is received from the
   * wheel encoders
   *
   * @param lb_speed wheel speeds in rpm
   * @param lf_speed wheel speeds in rpm
   * @param rb_speed wheel speeds in rpm
   * @param rf_speed wheel speeds in rpm
   */
  void _wheel_speeds_subscription_callback(double lb_speed, double lf_speed, double rb_speed,
                                           double rf_speed, double steering_angle);

  /**
   * @brief Executes:
   * - the prediction, validation and discovery steps of the EKF
   * - publication of localization
   * - publication of map
   *
   */
  void _timer_callback();

  /**
   * @brief publishes the localization ('vehicle_localization') to the topic
   * vehicle_location
   *
   */
  void _publish_localization();

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
   * @brief Converts the odometry data to translational and rotational velocities
   *
   * @param lb_speed wheel speeds in rpm
   * @param lf_speed wheel speeds in rpm
   * @param rb_speed wheel speeds in rpm
   * @param rf_speed wheel speeds in rpm
   */
  static MotionUpdate odometry_to_velocities_transform(double lb_speed,
                                                       [[maybe_unused]] double lf_speed,
                                                       double rb_speed,
                                                       [[maybe_unused]] double rf_speed,
                                                       double steering_angle);

 public:
  /**
   * @brief Construct a new LMNode object
   *
   * @param ekf Pointer to the EKF
   * @param map Pointer to the map
   * @param motion_update Pointer to the motion update
   * @param track_map Pointer to the track map
   * @param vehicle_state Pointer to the vehicle state
   * @param use_odometry Whether to use odometry or IMU
   *
   */
  LMNode(ExtendedKalmanFilter* ekf, Map* perception_map, MotionUpdate* motion_update,
         Map* track_map, VehicleState* vehicle_state, bool use_odometry);

  void set_mission(Mission mission);

  Mission get_mission();

  friend class Adapter;
  FRIEND_TEST(ODOMETRY_SUBSCRIBER, CONVERSION_TEST);
};

#endif  // SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_