#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "clustering/clustering.hpp"
#include "utils/grid_geometry.hpp"
#include "utils/grid_index.hpp"

/**
 * @brief Grid-based clustering algorithm implementation.
 * implementation.
 *
 * The GridClustering class is a concrete implementation of the Clustering interface, using a
 * grid-based algorithm to cluster point clouds based on spatial partitioning.
 *
 */
class GridClustering : public Clustering {
private:
  GridGeometry grid_geometry_;  ///< Geometry of the grid used for clustering.

public:
  /**
   * @brief Constructor for the GridClustering algorithm.
   *
   * @param grid_width Width of each grid cell.
   * @param max_points_per_cluster Maximum number of points allowed in each cluster.
   * @param min_points_per_cluster Minimum number of points required to form a cluster.
   */
  GridClustering(double grid_angle, double grid_radius, double start_augmentation,
                 double radius_augmentation, double fov);

  /**
   * @brief Clusters the input point cloud into groups using the GridClustering algorithm.
   *
   * This function implements the clustering method from the Clustering interface.
   *
   * @param point_cloud A shared pointer to a point cloud of type pcl::PointCloud<pcl::PointXYZI>.
   * @param clusters A pointer to a vector of shared pointers to point clouds to store the resulting
   * clusters.
   */
  void clustering(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                  std::vector<Cluster>* clusters) const override;
};