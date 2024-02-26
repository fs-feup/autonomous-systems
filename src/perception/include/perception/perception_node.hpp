#ifndef SRC_PERCEPTION_PERCEPTION_INCLUDE_PERCEPTION_PERCEPTION_NODE_HPP_
#define SRC_PERCEPTION_PERCEPTION_INCLUDE_PERCEPTION_PERCEPTION_NODE_HPP_

#include <string>
#include "custom_interfaces/msg/cone_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ground_removal/ransac.hpp"
#include "rclcpp/rclcpp.hpp"
#include "clustering/dbscan.hpp"

class Adapter;

/**
 * @class Perception
 * @brief Node for perception tasks, such as ground removal and cone detection.
 *
 * This class is a ROS 2 node that subscribes to a PointCloud2 topic, performs
 * ground removal using the specified GroundRemoval algorithm, and publishes
 * cone detection results on a custom topic.
 */
class Perception : public rclcpp::Node {
 private:
  GroundRemoval* groundRemoval; ///< Pointer to the GroundRemoval object.
  Adapter *adapter; /**< Adapter instance for external communication */
  Clustering* clustering;
  std::string mode = "fsds"; // Temporary, change as desired. TODO(andre): Make not hardcoded

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      _point_cloud_subscription;  ///< PointCloud2 subscription.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      _cones_publisher; ///< ConeArray publisher.

 public:
   /**
   * @brief Constructor for the Perception node.
   * @param groundRemoval Pointer to the GroundRemoval object.
   */

  Perception(GroundRemoval* groundRemoval, Clustering* clustering);

    /**
   * @brief Callback function for the PointCloud2 subscription.
   * @param msg The received PointCloud2 message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif


