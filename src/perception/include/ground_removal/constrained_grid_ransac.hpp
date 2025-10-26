#pragma once

#include "ground_removal/constrained_ransac_optimized.hpp"
#include "ground_removal/ground_removal.hpp"

/**
 * @class ConstrainedGridRANSAC
 * @brief Implementation of ground removal using a grid-based RANSAC algorithm.
 *
 * This class inherits from the GroundRemoval abstract class and provides
 * an implementation for ground removal using a grid-based RANSAC algorithm.
 */
class ConstrainedGridRANSAC : public GroundRemoval {
public:
  /**
   * @brief Constructor for ConstrainedGridRANSAC class.
   *
   * Initializes the ConstrainedGridRANSAC object with the specified parameters.
   *
   * @param epsilon Epsilon value for RANSAC algorithm.
   * @param n_tries Number of RANSAC iterations.
   * @param plane_angle_diff Maximum allowed angle deviation from the base plane.
   *
   *
   */
  ConstrainedGridRANSAC(const double epsilon, const int n_tries, const double plane_angle_diff);

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
  void ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                      const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                      const SplitParameters split_params) const override;

  /**
   * @brief Split the input point cloud into grids.
   *
   *
   * @param cloud The input point cloud to be split.
   * @param[out] grids Vector of vectors representing the grids.
   */
  void split_point_cloud(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                         std::vector<std::vector<pcl::PointCloud<PointXYZIR>::Ptr>>& grids,
                         const SplitParameters split_params) const;

private:
  ConstrainedRANSACOptimized _ransac_;  ///< Constrained RANSAC object for ground plane fitting.
};
