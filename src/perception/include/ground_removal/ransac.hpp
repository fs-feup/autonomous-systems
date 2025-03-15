#pragma once

#include <pcl/sample_consensus/ransac.h>

#include <string>
#include <utils/plane.hpp>
#include <utils/split_parameters.hpp>

#include "ground_removal/ground_removal.hpp"

/**
 * @class RANSAC
 * @brief Ground removal using the RANSAC algorithm.
 *
 * This class implements the GroundRemoval interface and performs ground removal
 * on a point cloud using the Random Sample Consensus (RANSAC) algorithm.
 */
class RANSAC : public GroundRemoval {
public:
  /**
   * @brief Constructor for the RANSAC ground removal algorithm.
   * @param epsilon Epsilon threshold for ground removal.
   * @param n_tries Number of RANSAC iterations.
   */
  RANSAC(const double epsilon, const int n_tries);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  RANSAC() = default;

  /**
   * @brief Perform ground removal using the RANSAC algorithm.
   *
   * This function implements the ground removal using the RANSAC algorithm
   * on the provided point cloud.
   *
   * @param point_cloud The input point cloud to be processed.
   * @param[out] ret The resulting point cloud after ground removal.
   */
  void ground_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane,
                      [[maybe_unused]] const SplitParameters split_params) const override;

private:
  double epsilon;  ///< Epsilon threshold for ground removal.
  int n_tries;     ///< Number of RANSAC iterations.
};
