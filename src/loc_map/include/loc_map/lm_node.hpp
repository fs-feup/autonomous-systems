#ifndef SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_
#define SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_

#include <gtest/gtest_prod.h>

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>

#include "adapter_loc_map/eufs.hpp"
#include "adapter_loc_map/fsds.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "kalman_filter/ekf.hpp"
#include "loc_map/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

class Adapter;

/**
 * @brief Class representing the main loc_map node responsible for publishing
 * the calculated vehicle localization and the map. As well as, subscribing and
 * interpreting information, such as the cone's position and colors, from the
 * perception module.
 *
 */
class LMNode : public rclcpp::Node {
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _perception_subscription;
  rclcpp::Publisher<custom_interfaces::msg::Pose>::SharedPtr _localization_publisher;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr _map_publisher;
  rclcpp::TimerBase::SharedPtr _timer; /**< timer */
  ExtendedKalmanFilter *_ekf;          /**< SLAM EKF object */
  ConeMap *_perception_map;
  MotionUpdate *_motion_update;
  ConeMap *_track_map;
  VehicleState *_vehicle_state;
  Mission _mission;
  bool _use_odometry;
  Adapter *adapter;
  std::string mode = "fsds";  // Temporary, change as desired. TODO(andre): Make not hardcoded

 public:
  /**
   * @brief Callback that updates everytime information
   * is received from the perception module
   *
   * @param msg Message containing the array of perceived cones
   */
  void _perception_subscription_callback(const custom_interfaces::msg::ConeArray msg);

  /**
   * @brief Function to be called everytime information is received from the
   * IMU
   *
   * @param rotational_velocity
   * @param acceleration_x
   * @param acceleration_y
   */
  void _imu_subscription_callback(double rotational_velocity, double acceleration_x,
                                  double acceleration_y);

  /**
   * @brief Function to be called everytime information is received from the
   * wheel encoders
   *
   * @param lb_speed wheel speeds in rpm
   * @param lf_speed wheel speeds in rpm
   * @param rb_speed wheel speeds in rpm
   * @param rf_speed wheel speeds in rpm
   * @param steering_angle steering angle in radians
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
  void _update_and_publish();

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

 public:
  /**
   * @brief LMNode constructor declaration
   *
   * @param ekf Pointer to the EKF
   * @param map Pointer to the map
   * @param motion_update Pointer to the motion update
   * @param track_map Pointer to the track map
   * @param vehicle_state Pointer to the vehicle state
   * @param use_odometry Whether to use odometry or IMU
   */
  LMNode(ExtendedKalmanFilter *ekf, ConeMap *perception_map, MotionUpdate *motion_update,
         ConeMap *track_map, VehicleState *vehicle_state, bool use_odometry);

  /**
   * @brief Mission setter
   *
   * @param mission New mission
   */
  void set_mission(Mission mission);

  // friend class Adapter;
  // FRIEND_TEST(ODOMETRY_SUBSCRIBER, CONVERSION_TEST);
};

#endif  // SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_