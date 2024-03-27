#ifndef SRC_PERCEPTION_PERCEPTION_INCLUDE_PERCEPTION_PERCEPTION_NODE_HPP_
#define SRC_PERCEPTION_PERCEPTION_INCLUDE_PERCEPTION_PERCEPTION_NODE_HPP_

#include <cone_validator/cylinder_validator.hpp>
#include <string>

#include "clustering/dbscan.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "ground_removal/ransac.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <utils/plane.hpp>
#include <vector>
#include "cone_validator/height_validator.hpp"

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
  GroundRemoval* groundRemoval;  ///< Pointer to the GroundRemoval object.
  Adapter* adapter;              /**< Adapter instance for external communication */
  Clustering* clustering;
  std::string mode = "test";  // Temporary, change as desired. TODO(andre): Make not hardcoded
  ConeDifferentiation* coneDifferentiator;  ///< Pointer to ConeDifferentiation object.
  Plane groundPlane;
  std::vector<ConeValidator*> coneValidators;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      _point_cloud_subscription;  ///< PointCloud2 subscription.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      _cones_publisher;  ///< ConeArray publisher.

  /**
   * @brief Publishes information about clusters (cones) using a custom ROS2 message.
   *
   * This function takes a vector of Cluster objects, extracts relevant information such as
   * centroid and color, and publishes this information using a custom ROS2 message type ConeArray.
   *
   * @param cones A reference to a vector of Cluster objects representing the clusters (cones) to be
   * published.
   */
  void publishCones(std::vector<Cluster>* cones);

 public:
  /**
   * @brief Constructor for the Perception node.
   * @param groundRemoval Pointer to the GroundRemoval object.
   * @param coneDifferentiator Pointer to ConeDifferentiation object
   */
  Perception(GroundRemoval* groundRemoval, Clustering* clustering,
             ConeDifferentiation* coneDifferentiator,
             const std::vector<ConeValidator*>& coneValidator);

  /**
   * @brief Callback function for the PointCloud2 subscription.
   * @param msg The received PointCloud2 message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif
