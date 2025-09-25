#pragma once
#include "ground_removal/ground_removal.hpp"

struct Ring {
  std::vector<int> indices;
};

struct Slice {
  std::vector<Ring> rings;
};

/**
 * @class RANSAC
 * @brief Ground removal using the RANSAC algorithm.
 *
 * This class implements the GroundRemoval interface and performs ground removal
 * on a point cloud using the Random Sample Consensus (RANSAC) algorithm.
 */
class Himmelsbach : public GroundRemoval {
public:
  /**
   * @brief Constructor for the Himmelsbach ground removal algorithm.
   * @param max_slope
   */
  Himmelsbach(const double max_slope);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  Himmelsbach() = default;

  /**
   * @brief Perform ground removal using the Himmelsbach algorithm.
   *
   * This function implements the ground removal using the Himmelsbach algorithm
   * on the provided point cloud.
   *
   * @param point_cloud The input point cloud to be processed.
   * @param[out] ret The resulting point cloud after ground removal.
   */
  void ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                      const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                      const SplitParameters split_params) const override;

private:
  double max_slope;  ///< Maximum slope for Himmelsbach algorithm.

  void split_point_cloud(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                         std::vector<Slice>& splited_cloud,
                         const SplitParameters split_params) const;
};
