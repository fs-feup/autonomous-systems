#pragma once

#include <random>

#include "ground_removal/ground_removal.hpp"

/**
 * @class Constrained RANSAC
 * @brief Ground removal using the Constrained RANSAC algorithm.
 *
 * This class implements the GroundRemoval interface and performs ground removal
 * on a point cloud using the Constrained Random Sample Consensus (RANSAC) algorithm.
 */
class ConstrainedRANSAC : public GroundRemoval {
public:
  /**
   * @brief Constructor for the RANSAC ground removal algorithm.
   * @param epsilon Epsilon threshold for ground removal.
   * @param n_tries Number of RANSAC iterations.
   * @param plane_angle_diff Maximum allowed angle deviation from the base plane.
   */
  ConstrainedRANSAC(const double epsilon, const int n_tries, const double plane_angle_diff);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  ConstrainedRANSAC() = default;

  /**
   * @brief Perform ground removal using the RANSAC algorithm.
   *
   * This function implements the ground removal using the RANSAC algorithm
   * on the provided point cloud.
   *
   * @param point_cloud The input point cloud to be processed.
   * @param[out] ret The resulting point cloud after ground removal.
   * @param[out] plane The best-fit plane found by RANSAC.
   * @param split_params Split parameters (unused in this implementation).
   */
  void ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                      const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                      [[maybe_unused]] const SplitParameters split_params) const override;

private:
  double epsilon;           ///< Epsilon threshold for ground removal.
  int n_tries;              ///< Number of RANSAC iterations.
  double plane_angle_diff;  ///< Maximum allowed angle deviation from base plane.

  /**
   * @brief Calculate the best-fit plane using RANSAC algorithm.
   * @param point_cloud Input point cloud.
   * @param target_lane
   */
  Plane calculate_plane(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                        const Plane& target_plane) const;

  /**
   * @brief Fit a plane to three points.
   * @param points Vector of exactly 3 points.
   * @return Plane fitted to the three points.
   */
  Plane fit_plane_to_points(const std::vector<PointXYZIR>& points) const;

  /**
   * @brief Calculate the distance of a point to the plane.
   *
   * @param point
   * @return Distance from point to plane.
   */
  double distance_to_plane(const PointXYZIR& point, const Plane& plane) const;

  double calculate_angle_difference(const Plane& plane1, const Plane& plane2) const;

  /**
   * @brief Pick 3 unique random indices from the range [0, max_index).
   * @param max_index The exclusive upper bound for index selection.
   * @param gen Random number generator.
   * @return Vector of 3 unique random indices.
   */
  std::vector<int> pick_3_random_indices(int max_index, std::mt19937& gen) const;
};