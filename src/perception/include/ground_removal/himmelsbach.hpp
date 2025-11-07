#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utils/ground_grid.hpp>
#include <utils/lidar_point.hpp>
#include <utils/split_parameters.hpp>
#include <vector>

#include "ground_removal/ground_removal.hpp"
#include "rclcpp/rclcpp.hpp"

int constexpr NUM_RINGS = 40;

/**
 * @struct Ring
 * @brief Stores indices of points in a single ring and the minimum height index.
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
  Himmelsbach(const double max_slope, const double min_slope, const double slope_reduction_m,
              const double start_reduction, const double initial_alpha,
              const double alpha_augmentation_m, const double start_augmentation,
              SplitParameters split_params);

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
  double max_slope_;
  double min_slope_;
  double slope_reduction_m_;
  double start_reduction_;
  double initial_alpha_;
  double alpha_augmentation_m_;
  double start_augmentation_;
  SplitParameters split_params_;
  std::shared_ptr<std::vector<Slice>> slices_;

  void process_slice(const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
                     sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud,
                     size_t slice_idx, GroundGrid& ground_grid) const;

  void split_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud) const;
};
