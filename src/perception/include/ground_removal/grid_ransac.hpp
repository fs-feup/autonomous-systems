#include <pcl/sample_consensus/ransac.h>

#include <string>
#include <utils/plane.hpp>

#include "ground_removal/ground_removal.hpp"
#include "ground_removal/ransac.hpp"

/**
 * @class GridRANSAC
 * @brief Implementation of ground removal using a grid-based RANSAC algorithm.
 *
 * This class inherits from the GroundRemoval abstract class and provides
 * an implementation for ground removal using a grid-based RANSAC algorithm.
 */
class GridRANSAC : public GroundRemoval {
 private:
  RANSAC ransac; ///< RANSAC object for ground plane fitting.
  int n_angular_grids; ///< Number of angular grids in the algorithm.
  double radius_resolution; ///< Resolution of the radius for grid splitting.

 public:
  /**
   * @brief Constructor for GridRANSAC class.
   *
   * Initializes the GridRANSAC object with the specified parameters.
   *
   * @param epsilon Epsilon value for RANSAC algorithm.
   * @param n_tries Number of RANSAC iterations.
   * @param n_angular_grids Number of angular grids for grid-based segmentation.
   * @param radius_resolution Resolution of the radius for grid splitting.
   */
  GridRANSAC(double epsilon, int n_tries, int n_angular_grids, double radius_resolution);

  /**
   * @brief Perform ground removal on the input point cloud using grid-based RANSAC.
   *
   * This method implements the groundRemoval virtual function from the GroundRemoval interface.
   * It segments the input point cloud into grids and applies RANSAC on each grid for ground removal.
   *
   * @param point_cloud The input point cloud to be processed.
   * @param[out] ret The resulting point cloud after ground removal.
   * @param plane The estimated ground plane model.
   */
  void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane) const override;

  /**
   * @brief Get the furthest point from the input point cloud.
   *
   * This method calculates the furthest point from the input point cloud.
   *
   * @param cloud The input point cloud.
   * @return The distance of the furthest point from the origin.
   */
  static double get_furthest_point(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  /**
   * @brief Split the input point cloud into grids.
   *
   * This method splits the input point cloud into grids based on the specified parameters.
   *
   * @param cloud The input point cloud to be split.
   * @param[out] grids Vector of vectors representing the grids.
   */
  void split_point_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>& grids) const;
};
