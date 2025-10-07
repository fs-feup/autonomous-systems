#pragma once
#include "ground_removal/ground_removal.hpp"

int constexpr NUM_RINGS = 40;

struct Ring {
  // Each pair contains the index of the point in the original point cloud and a boolean indicating
  // whether it is ground or not
  std::vector<std::pair<int, bool>> indices;
  int indices_min_height_idx =
      -1;  // Index in indices vector of the point with minimum height in the ring

  Ring() = default;
};

struct Slice {
  std::vector<Ring> rings{NUM_RINGS};

  Slice() = default;
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
   * @param epsilon
   * @param adjacent_slices Number of adjacent slices to use for ground height
   */
  Himmelsbach(const double max_slope, const double epsilon, const int adjacent_slices);

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
  double max_slope;     ///< Maximum slope for Himmelsbach algorithm.
  double epsilon;       ///< Maximum distance from a ground point to still be considered ground
  int adjacent_slices;  ///< Number of adjacent slices to use for ground height
                        ///< reference

  void split_point_cloud(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                         std::vector<Slice>& splited_cloud,
                         const SplitParameters split_params) const;

  void process_slice(const pcl::PointCloud<PointXYZIR>::Ptr& cloud, Slice& slice,
                     const float ground_height_reference) const;

  float calculate_height_reference(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                   const std::vector<Slice>& splited_cloud,
                                   const int cur_slice_idx) const;

  int find_closest_ring_min_height_idx(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                       const Slice& slice) const;
};
