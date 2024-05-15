#ifndef SRC_PERCEPTION_PERCEPTION_INCLUDE_PERCEPTION_PERCEPTION_NODE_HPP_
#define SRC_PERCEPTION_PERCEPTION_INCLUDE_PERCEPTION_PERCEPTION_NODE_HPP_

#include <cone_evaluator/cone_evaluator.hpp>
#include <cone_validator/cylinder_validator.hpp>
#include <string>
#include <utils/plane.hpp>
#include <vector>

#include "clustering/dbscan.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include "cone_validator/height_validator.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "ground_removal/ransac.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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
  std::shared_ptr<GroundRemoval> groundRemoval;  ///< Shared pointer to the GroundRemoval object.
  std::shared_ptr<Adapter> adapter;              /**< Shared pointer to Adapter instance for external communication */
  std::shared_ptr<Clustering> clustering;
  std::string mode;
  std::shared_ptr<ConeDifferentiation> coneDifferentiator;  ///< Shared pointer to ConeDifferentiation object.
  Plane groundPlane;
  std::vector<std::shared_ptr<ConeValidator>> cone_validators;
  std::shared_ptr<ConeEvaluator> coneEvaluator;

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
   * @param groundRemoval Shared pointer to the GroundRemoval object.
   * @param clustering Shared pointer to the Clustering object.
   * @param coneDifferentiator Shared pointer to the ConeDifferentiation object
   * @param coneValidator Vector of shared pointers to ConeValidator objects.
   * @param coneEvaluator Shared pointer to the ConeEvaluator object.
   * @param mode String representing the mode.
   */
  Perception(std::shared_ptr<GroundRemoval> groundRemoval, std::shared_ptr<Clustering> clustering,
             std::shared_ptr<ConeDifferentiation> coneDifferentiator,
             const std::vector<std::shared_ptr<ConeValidator>>& cone_validators, 
             std::shared_ptr<ConeEvaluator> coneEvaluator, std::string mode);

  /**
   * @brief Callback function for the PointCloud2 subscription.
   * @param msg The received PointCloud2 message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif
