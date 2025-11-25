#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "center_calculation/centroid_calculation.hpp"
#include "center_calculation/circunference_center_calculation.hpp"
#include "utils/lidar_point.hpp"

/**
 * @brief Represents a cluster of 3D points using PCL (Point Cloud Library).
 */
class Cluster {
public:
  /**
   * @brief Constructor for the Cluster class.
   */
  explicit Cluster(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                   const std::vector<int>& point_indices);

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
  void set_point_indices(const std::vector<int>& new_point_indices);

  /**
   * @brief Get the Point Cloud data of the cluster.
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr representing the Point Cloud data.
   */
  const std::vector<int>& get_point_indices();

  /**
   * @brief Get the Point Cloud data of the cluster.
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr representing the Point Cloud data.
   */
  const sensor_msgs::msg::PointCloud2::SharedPtr& get_point_cloud();

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

  /**
   * @brief Set the z score object on the x and y axis
   *
   * @param mean_x Mean of the x on the point's distribution
   * @param std_dev_x Standard Deviation of the x on the point's distribution
   * @param mean_y Mean of the y on the point's distribution
   * @param std_dev_y Standard Deviation of the y on the point's distribution
   */
  void set_z_score(double mean_x, double std_dev_x, double mean_y, double std_dev_y);

  /**
   * @brief Get the z score on x-axis of an object
   *
   * @return double Z-score on x axis
   */
  double get_z_score_x() const;

  /**
   * @brief Get the z score on y-axis of an object
   *
   * @return double Z-score on y axis
   */
  double get_z_score_y() const;

  /**
   * @brief Calculates the mean and standard deviation on x and y axis.
   *
   * @param clusters Clusters to apply get the statistical values
   * @return std::tuple<double, double, double, double> (mean x, mean y, standard deviation x,
   * standard deviation y)
   */
  static std::tuple<double, double, double, double> calculate_mean_and_std_dev(
      std::vector<Cluster>& clusters);

  /**
   * @brief Set the z scores object on every cluster of the vector
   *
   * @param clusters Vector of clusters in which the a-scores will be applied individually
   */
  static void set_z_scores(std::vector<Cluster>& clusters);

  /**
   * @brief Get cluster's corresponding cone size.
   * @return bool representing size .
   */
  bool get_is_large();

  /**
   * @brief Set cluster as a contender for a large cone
   */
  void set_is_large();

private:
  const sensor_msgs::msg::PointCloud2::SharedPtr
      _point_cloud_;                   ///< Point cloud data for the cluster.
  std::vector<int> _point_indices_;    ///< Indices of points in the cluster.
  std::string _color_ = "undefined";   ///< Color associated with the cluster.
  Eigen::Vector4f _centroid_;          ///< Centroid of the cluster.
  Eigen::Vector4f _center_;            ///< Center of the cone's cluster.
  bool _centroid_is_defined_ = false;  ///< Flag indicating whether the centroid is defined or not.
  bool _center_is_defined_ = false;    ///< Flag indicating whether the center is defined or not.
  double _confidence_ = 0;             ///< Confidence on the cluster to be (or not) a cone
  double _z_score_x_ = 0;
  double _z_score_y_ = 0;
  bool _is_large_ = false;  ///< Flag indicating the size of the cluster :
  ///< true = large cluster.
  ///< false = small = cluster.

  static constexpr auto center_calculator =
      CircunferenceCenterCalculator();  ///< Calculates the center of the cone
  static constexpr auto centroid_calculator =
      CentroidCalculator();  ///< Calculates the centroid of the cluster
};
