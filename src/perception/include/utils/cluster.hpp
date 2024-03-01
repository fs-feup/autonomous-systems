#ifndef CLUSTER_HPP
#define CLUSTER_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/centroid.hpp>
#include <string>

/**
 * @brief Represents a cluster of 3D points using PCL (Point Cloud Library).
 */
class Cluster {
 private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud; ///< Pointer to the Point Cloud data.
    std::string color; ///< Color associated with the cluster.
    Eigen::Vector4f centroid; ///< Centroid of the cluster.
    bool centroidIsDefined; ///< Flag indicating whether the centroid is defined or not.

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
     * @brief Get the color associated with the cluster.
     * @return std::string representing the color.
     */
    std::string getColor();

    /**
     * @brief Set the color for the cluster.
     * @param new_color The new color to be set.
     */
    void setColor(std::string new_color);

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
};

#endif // CLUSTER_HPP
