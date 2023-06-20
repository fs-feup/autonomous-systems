#ifndef SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_PUBLISHER_HPP_
#define SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_PUBLISHER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>

#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "loc_map/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief Class for the node responsible for publishing the localization and the
 * map of the vehicle
 *
 */
class LMPublisher : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<custom_interfaces::msg::Pose>::SharedPtr _localization_publisher;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr _mapping_publisher;
  VehicleState* _vehicle_state;
  Map* _track_map;

  /**
   * @brief timer callback function for publishing the localization and the map
   *
   */
  void _timer_callback();

  /**
   * @brief publishes the localization ('vehicle_localization') to the topic
   * vehicle_location
   *
   * @param vehicle_pose
   */
  void _publish_localization(VehicleState vehicle_state);

  /**
   * @brief publishes the map ('track_map') to the topic track_map
   *
   * @param track_map
   */
  void _publish_map(Map track_map);

 public:
  /**
   * @brief Construct a new LMPublisher object
   *
   */
  LMPublisher(Map* track_map, VehicleState* state);
};

#endif  // SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_PUBLISHER_HPP_