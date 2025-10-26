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
 * @class Himmelsbach
 * @brief Ground removal using the Himmelsbach algorithm.
 *
 * This class implements the GroundRemoval interface and performs ground removal
 * on a point cloud using the Himmelsbach algorithm.
 */
class Himmelsbach : public GroundRemoval {
public:
  /**
   * @brief Constructor for the Himmelsbach ground removal algorithm.
   * @param max_slope Maximum slope between ground points
   * @param epsilon Maximum distance from a ground point to still be considered ground
   * @param adjacent_slices Number of adjacent slices to use for ground height reference
   * @param slope_reduction Slope reduction for farther rings
   * @param distance_reduction Distance interval for slope reduction
   * @param min_slope Minimum slope for the distance reduction
   */
  Himmelsbach(const double max_slope, const double epsilon, const int adjacent_slices,
              const double slope_reduction, const double distance_reduction,
              const double min_slope);

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
  double max_slope;        ///< Maximum slope between ground points
  double epsilon;          ///< Maximum distance from a ground point to still be considered ground
  int adjacent_slices;     ///< Number of adjacent slices to use for ground height reference
  double slope_reduction;  ///< Slope reduction for farther rings
  double distance_reduction;  ///< Distance interval for slope reduction
  double min_slope;           ///< Minimum slope for the distance reduction

  /**
   * @brief Split the point cloud into slices and rings. Using the ring information from the point
   * cloud to not compute the azimuth angle.
   *
   * @param cloud The input point cloud to be split.
   * @param splited_cloud The resulting vector of slices after splitting.
   * @param split_params Parameters for splitting the point cloud.
   */
  void split_point_cloud(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                         std::vector<Slice>& splited_cloud,
                         const SplitParameters split_params) const;

  /**
   * @brief Process a single slice to identify ground points based on slope between rings and height
   * reference.
   *
   * @param cloud The input point cloud.
   * @param slice The slice to be processed.
   * @param ground_height_reference The reference height for ground points in the slice.
   */
  void process_slice(const pcl::PointCloud<PointXYZIR>::Ptr& cloud, Slice& slice,
                     const double ground_height_reference) const;

  /**
   * @brief Calculate the ground height reference for a given slice based on adjacent slices.
   * @param cloud The input point cloud.
   * @param splited_cloud The vector of all slices.
   * @param cur_slice_idx The index of the current slice for which to calculate the reference.
   */
  double calculate_height_reference(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                    const std::vector<Slice>& splited_cloud,
                                    const int cur_slice_idx) const;

  /**
   * @brief Find the index of the point with the minimum height in the closest non-empty ring of a
   * slice.
   * @param slice The slice to be searched.
   */
  int find_closest_ring_min_height_idx(const Slice& slice) const;
};
