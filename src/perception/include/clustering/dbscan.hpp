#pragma once
#include "clustering/clustering.hpp"

/**
 * @brief DBSCAN (Density-Based Spatial Clustering of Applications with Noise) clustering algorithm
 * implementation.
 *
 * The DBSCAN class is a concrete implementation of the Clustering interface, using the DBSCAN
 * algorithm to cluster point clouds based on density and distance thresholds.
 *
 */
class DBSCAN : public Clustering {
private:
  int min_cluster_size;  ///< Minimum number of points required to form a cluster.
  /**
   * @brief Maximum distance between points to be considered as neighbors.
   *
   */
  double neighbours_dist_threshold;

public:
  /**
   * @brief Constructor for the DBSCAN clustering algorithm.
   *
   * @param min_cluster_size Minimum number of points required to form a cluster.
   * @param neighbours_dist_threshold Maximum distance between points to be considered as neighbors.
   */
  DBSCAN(int min_cluster_size, double neighbours_dist_threshold);

  /**
   * @brief Clusters the input point cloud into groups using the DBSCAN algorithm.
   *
   * This function implements the clustering method from the Clustering interface.
   *
   * @param point_cloud A shared pointer to a point cloud of type pcl::PointCloud<pcl::PointXYZI>.
   * @param clusters A pointer to a vector of shared pointers to point clouds to store the resulting
   * clusters.
   */
  void clustering(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                  std::vector<Cluster>* clusters) const override;
};