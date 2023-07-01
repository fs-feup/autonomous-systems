#ifndef SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_
#define SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "loc_map/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Class for the node responsible for subscribing the cones' colors and
 * coordinates published by the perception module.
 */
class LMSubscriber : public rclcpp::Node {
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _perception_subscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription;
  // rclcpp::Subscription<
  Map* _map;
  ImuUpdate* _imu_update;

  /**
   * @brief function to be called everytime information is received from the
   * perception module
   */
  void _perception_subscription_callback(const custom_interfaces::msg::ConeArray message);

  /**
   * @brief function to be called everytime information is received from the
   * imu
   */
  void _imu_subscription_callback(const sensor_msgs::msg::Imu message);

 public:
  /**
   * @brief Construct a new LMSubscriber object
   *
   */
  LMSubscriber(Map* map, ImuUpdate* imu_update);
};

#endif  // SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_SUBSCRIBER_HPP_