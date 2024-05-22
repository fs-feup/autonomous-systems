#ifndef CLUSTER_HPP
#define CLUSTER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "center_calculation/circunferece_center_calculation.hpp"

#include <pcl/common/impl/centroid.hpp>
#include <string>

/**
 * @brief Represents a cluster of 3D points using PCL (Point Cloud Library).
 */
class Cluster {
 private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud_;  ///< Pointer to the Point Cloud data.
  std::string _color_;                                 ///< Color associated with the cluster.
  Eigen::Vector4f _centroid_;                          ///< Centroid of the cluster.
  Eigen::Vector4f _center_;                          ///< Center of the cone's cluster.
  bool _centroid_is_defined_ = false;  ///< Flag indicating whether the centroid is defined or not.
  bool _center_is_defined_ = false; ///< Flag indicating whether the center is defined or not.
  double _confidence_ = 0; ///< Confidence on the cluster to be (or not) a cone
  static constexpr auto center_calculator = CircunferenceCenterCalculation(); ///< Calculates the center of the cone

 public:
  /**
   * @brief Constructor for the Cluster class.
   * @param point_cloud Pointer to the Point Cloud data of the cluster.
   */
  explicit Cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);

  /**
   * @brief Get the centroid of the cluster.
   * @return Eigen::Vector4f representing the centroid.
   */
  Eigen::Vector4f getCentroid();

  /**
   * @brief Get the Center of the cone's cluster
   * 
   * @return Eigen::Vector4f representing the center
   */
  Eigen::Vector4f get_center(Plane& plane);

  /**
   * @brief Get the color associated with the cluster.
   * @return std::string representing the color.
   */
  std::string getColor();

  /**
   * @brief Set the color for the cluster.
   * @param new_color The new color to be set.
   */
  void setColor(const std::string& new_color);

  /**
   * @brief Set the Point Cloud data for the cluster.
   * @param new_point_cloud Pointer to the new Point Cloud data.
   */
  void setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud);

  /**
   * @brief Get the Point Cloud data of the cluster.
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr representing the Point Cloud data.
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud();

  /**
   * @brief Set the Confidence of the cluster to be or not to be a cone
   * 
   * @param newConfidence The new Confidence of the cluster
   */
  void setConfidence(double newConfidence);

  /**
   * @brief Get the Confidence of the cluster to be (or not to be) a cone
   * 
   * @return double Cluster's confidence
   */
  double getConfidence();
};

#endif  // CLUSTER_HPP
