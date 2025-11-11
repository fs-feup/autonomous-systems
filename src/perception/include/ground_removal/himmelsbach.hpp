#pragma once
#include "ground_removal/ground_removal.hpp"

int constexpr NUM_RINGS = 40;

/**
 * @struct Ring
 * @brief Stores indices of points in a single ring.
 */
struct Ring {
  std::vector<int> indices;

  Ring() = default;
};

/**
 * @struct Slice
 * @brief Represents a horizontal slice of the LiDAR scan.
 */
struct Slice {
  std::vector<Ring> rings{NUM_RINGS};

  Slice() = default;
};

/**
 * @class Himmelsbach
 * @brief Ground removal using the Himmelsbach algorithm.
 *
 * Implements GroundRemoval interface for LiDAR point clouds.
 */
class Himmelsbach : public GroundRemoval {
public:
  /**
   * @brief Constructor for Himmelsbach ground removal.
   * @param grid_angle Angle increment for slicing the point cloud.
   * @param max_slope Maximum slope angle to consider a point as ground.
   * @param initial_alpha Initial distance threshold for ground classification.
   * @param alpha_augmentation_m Increase of alpha by meter.
   * @param start_augmentation Range at which distance threshold starts to increase.
   * @param trim_params Parameters for trimming the point cloud.
   */
  Himmelsbach(const double grid_angle, const double max_slope, const double initial_alpha,
              const double alpha_augmentation_m, const double start_augmentation,
              TrimmingParameters trim_params);

  Himmelsbach() = default;

  /**
   * @brief Perform ground removal on a point cloud.
   * @param trimmed_point_cloud Input point cloud (trimmed).
   * @param ground_removed_point_cloud Output point cloud with ground removed.
   * @param plane Plane information (not estimated by Himmelsbach, set to default).
   * @param split_params Parameters for slicing the point cloud.
   */
  void ground_removal(const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
                      sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud,
                      GroundGrid& ground_grid) const override;

private:
  double grid_angle_;     // Angle increment for slicing the point cloud
  double max_slope_;      // Maximum slope to consider a point as ground candidate
  double initial_alpha_;  // Initial distance threshold for ground classification of candidates
  double alpha_augmentation_m_;  // Increase of alpha by meter
  double start_augmentation_;    // Range at which distance threshold starts to increase
  TrimmingParameters trim_params_;
  std::shared_ptr<std::vector<Slice>>
      slices_;  // Vector containing all slices, initialized in constructor

  /**
   * @brief Process a single slice for ground removal.
   * @param trimmed_point_cloud Input point cloud (trimmed).
   * @param ground_removed_point_cloud Output point cloud with ground removed.
   * @param slice_idx Index of the slice to process.
   * @param ground_grid Ground grid to update with ground heights.
   */
  void process_slice(const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
                     sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud,
                     size_t slice_idx, GroundGrid& ground_grid) const;

  /**
   * @brief Split the input point cloud into slices and rings. It will populate the `slices_`
   * member.
   * @param input_cloud Input point cloud to be split.
   */
  void split_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud) const;
};
