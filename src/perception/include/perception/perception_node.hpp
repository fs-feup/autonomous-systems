#pragma once

#include <cone_evaluator/cone_evaluator.hpp>
#include <cone_validator/cylinder_validator.hpp>
#include <string>
#include <utils/plane.hpp>
#include <vector>

#include "clustering/dbscan.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include "cone_evaluator/distance_predict.hpp"
#include "cone_validator/height_validator.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "ground_removal/grid_ransac.hpp"
#include "ground_removal/ransac.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "icp/icp.hpp"

struct PerceptionParameters {
  std::shared_ptr<GroundRemoval> ground_removal_;
  std::shared_ptr<DBSCAN> clustering_;
  std::shared_ptr<LeastSquaresDifferentiation> cone_differentiator_;
  std::vector<std::shared_ptr<ConeValidator>> cone_validators_;
  std::shared_ptr<DistancePredict> distance_predict_;
  std::shared_ptr<ICP> icp_;
  std::string adapter_;
};

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
  std::shared_ptr<GroundRemoval> _ground_removal_;  ///< Shared pointer to the GroundRemoval object.
  std::shared_ptr<Clustering> _clustering_;
  std::shared_ptr<ConeDifferentiation>
      _cone_differentiator_;  ///< Shared pointer to ConeDifferentiation object.
  Plane _ground_plane_;
  std::vector<std::shared_ptr<ConeValidator>> _cone_validators_;
  std::shared_ptr<ConeEvaluator> _cone_evaluator_;
  std::string _mode_;
  std::shared_ptr<ICP> _icp_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      _point_cloud_subscription;  ///< PointCloud2 subscription.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      _cones_publisher;  ///< ConeArray publisher.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_marker_array;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      _ground_removed_publisher;

  /**
   * @brief Publishes information about clusters (cones) using a custom ROS2 message.
   *
   * This function takes a vector of Cluster objects, extracts relevant information such as
   * centroid and color, and publishes this information using a custom ROS2 message type
   * ConeArray.
   *
   * @param cones A reference to a vector of Cluster objects representing the clusters (cones)
   * to be published.
   */
  void publishCones(std::vector<Cluster>* cones);

public:
  /**
   * @brief Constructor for the Perception node.
   * @param params The parameters for perception.
   */
  explicit Perception(const PerceptionParameters& params);

  /**
   * @brief Callback function for the PointCloud2 subscription.
   * @param msg The received PointCloud2 message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

};
