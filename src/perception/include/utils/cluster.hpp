#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "center_calculation/circunferece_center_calculation.hpp"

#include <pcl/common/impl/centroid.hpp>
#include <string>
#include <tuple>

/**
 * @brief Represents a cluster of 3D points using PCL (Point Cloud Library).
 */
class Cluster {
 private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud_;  ///< Pointer to the Point Cloud data.
  std::string _color_ = "undefined";  ///< Color associated with the cluster.
  Eigen::Vector4f _centroid_;                          ///< Centroid of the cluster.
  Eigen::Vector4f _center_;                          ///< Center of the cone's cluster.
  bool _centroid_is_defined_ = false;  ///< Flag indicating whether the centroid is defined or not.
  bool _center_is_defined_ = false; ///< Flag indicating whether the center is defined or not.
  double _confidence_ = 0; ///< Confidence on the cluster to be (or not) a cone
  static constexpr auto center_calculator = CircunferenceCenterCalculation(); ///< Calculates the center of the cone
  double _z_score_x_ = 0;
  double _z_score_y_ = 0;

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
  Eigen::Vector4f get_centroid();

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
  std::string get_color();

  /**
   * @brief Set the color for the cluster.
   * @param new_color The new color to be set.
   */
  void set_color(const std::string& new_color);

  /**
   * @brief Set the Point Cloud data for the cluster.
   * @param new_point_cloud Pointer to the new Point Cloud data.
   */
  void set_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud);

  /**
   * @brief Get the Point Cloud data of the cluster.
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr representing the Point Cloud data.
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_point_cloud();

  /**
   * @brief Set the Confidence of the cluster to be or not to be a cone
   * 
   * @param newConfidence The new Confidence of the cluster
   */
  void set_confidence(double newConfidence);

  /**
   * @brief Get the Confidence of the cluster to be (or not to be) a cone
   * 
   * @return double Cluster's confidence
   */
  double get_confidence();

  void set_z_score(double mean_x, double std_dev_x, double mean_y, double std_dev_y);

  double get_z_score_x();

  double get_z_score_y();

  static std::tuple<double, double, double, double> calculate_mean_and_std_dev(std::vector<Cluster>& clusters);

  static void set_z_scores(std::vector<Cluster>& clusters);

};
