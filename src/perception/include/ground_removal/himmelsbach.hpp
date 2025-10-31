#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utils/lidar_point.hpp>
#include <utils/plane.hpp>
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
  Himmelsbach(const double max_slope, const double slope_reduction, const double distance_reduction,
              const double min_slope, const int ground_reference_slices, const double epsilon,
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
                      Plane& plane) const override;

private:
  double max_slope;
  double slope_reduction;
  double distance_reduction;
  double min_slope;
  int ground_reference_slices;
  double epsilon;
  int max_points_per_ring;
  std::shared_ptr<std::vector<Slice>> slices_;
  SplitParameters split_params;

  void process_slice(const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
                     sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud,
                     size_t slice_idx) const;

  void split_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud) const;

  double calculate_ground_reference(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                    int slice_idx) const;

  float find_closest_ring_min_height(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                     int slice_idx) const;
};
