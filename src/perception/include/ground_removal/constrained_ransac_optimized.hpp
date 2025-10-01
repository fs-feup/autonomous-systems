#pragma once

#include "ground_removal/ground_removal.hpp"

/**
 * @class ConstrainedRANSACOptimized
 * @brief Constrained RANSAC with simple optimizations / pruning heuristics.
 *
 * This class implements the GroundRemoval interface and performs ground removal
 * on a point cloud using a constrained RANSAC variant that can prune points
 * when candidate planes are rejected by the angle constraint.
 */
class ConstrainedRANSACOptimized : public GroundRemoval {
public:
  /**
   * @brief Construct optimized constrained RANSAC.
   * @param epsilon distance threshold for RANSAC in meters.
   * @param n_tries maximum RANSAC iterations.
   * @param plane_angle_diff maximum allowed angle difference (degrees) from target plane.
   */
  ConstrainedRANSACOptimized(const double epsilon, const int n_tries,
                             const double plane_angle_diff);

  ConstrainedRANSACOptimized() = default;

  /**
   * @brief Perform ground removal.
   * @param point_cloud input point cloud.
   * @param[out] ret output cloud containing non-ground points.
   * @param[out] plane plane chosen (or left as provided/default when no candidate accepted).
   * @param split_params grid / split parameters (may be unused).
   */
  void ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                      const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                      [[maybe_unused]] const SplitParameters split_params) const override;

private:
  double epsilon;           ///< RANSAC distance threshold.
  int n_tries;              ///< RANSAC max iterations.
  double plane_angle_diff;  ///< Max allowed angle difference in degrees.

  /**
   * @brief Compute absolute distance from point to plane.
   */
  double distance_to_plane(const PointXYZIR& point, const Plane& plane) const;

  /**
   * @brief Compute acute angle difference (degrees) between two plane normals.
   */
  double calculate_angle_difference(const Plane& plane1, const Plane& plane2) const;
};