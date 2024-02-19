#include "custom_interfaces/msg/cone_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ground_removal/ransac.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "clustering/dbscan.hpp"
#include <vector>

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
  Clustering* clustering;
  ConeDifferentiation* coneDifferentiator; ///< Pointer to ConeDifferentiation object.


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      _point_cloud_subscription;  ///< PointCloud2 subscription.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      _cones_publisher; ///< ConeArray publisher.

  /**
   * @brief Callback function for the PointCloud2 subscription.
   * @param msg The received PointCloud2 message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void publishCones(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* clusters);

 public:
   /**
   * @brief Constructor for the Perception node.
   * @param groundRemoval Pointer to the GroundRemoval object.
   * @param coneDifferentiator Pointer to ConeDifferentiation object
   */
  Perception(GroundRemoval* groundRemoval, Clustering* clustering,
             ConeDifferentiation* coneDifferentiator);
};