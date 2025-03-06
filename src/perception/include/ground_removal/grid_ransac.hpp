#include <pcl/sample_consensus/ransac.h>

#include <string>
#include <utils/pcl_split_parameters.hpp>
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
public:
  /**
   * @brief Constructor for GridRANSAC class.
   *
   * Initializes the GridRANSAC object with the specified parameters.
   *
   * @param epsilon Epsilon value for RANSAC algorithm.
   * @param n_tries Number of RANSAC iterations.
   *
   */
  GridRANSAC(double epsilon, int n_tries);

  /**
   * @brief Perform ground removal on the input point cloud using grid-based RANSAC.
   *
   * This method implements the groundRemoval virtual function from the GroundRemoval interface.
   * It segments the input point cloud into grids and applies RANSAC on each grid for ground
   * removal.
   *
   * @param point_cloud The input point cloud to be processed.
   * @param[out] ret The resulting point cloud after ground removal.
   * @param plane The estimated ground plane model.
   */
  void ground_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane,
                      PclSplitParameters split_params) const override;

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
  void split_point_cloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>& grids) const;

private:
  RANSAC _ransac_;  ///< RANSAC object for ground plane fitting.
};
