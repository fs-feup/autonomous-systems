#include "ground_removal/himmelsbach.hpp"

Himmelsbach::Himmelsbach(const double max_slope, const double epsilon, const int adjacent_slices,
                         const double slope_reduction, const double distance_reduction,
                         const double min_slope)
    : max_slope(max_slope),
      epsilon(epsilon),
      adjacent_slices(adjacent_slices),
      slope_reduction(slope_reduction),
      distance_reduction(distance_reduction),
      min_slope(min_slope) {}

void Himmelsbach::ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                                 const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                                 [[maybe_unused]] const SplitParameters split_params) const {
  // Slip the point cloud into slices
  std::vector<Slice> splited_cloud;
  split_point_cloud(point_cloud, splited_cloud, split_params);

  // For each slice, calculate the ground height reference and process the slice
  for (int i = 0; i < static_cast<int>(splited_cloud.size()); i++) {
    double ground_height_reference = calculate_height_reference(point_cloud, splited_cloud, i);
    process_slice(point_cloud, splited_cloud[i], ground_height_reference);
  }

  ret->points.clear();
  ret->height = 1;
  ret->width = 0;
  ret->is_dense = false;

  // Collect non-ground points
  for (int i = 0; i < static_cast<int>(splited_cloud.size()); ++i) {
    for (int j = 0; j < static_cast<int>(splited_cloud[i].rings.size()); ++j) {
      for (int k = 0; k < static_cast<int>(splited_cloud[i].rings[j].indices.size()); ++k) {
        int index = splited_cloud[i].rings[j].indices[k].first;
        bool is_ground = splited_cloud[i].rings[j].indices[k].second;
        if (!is_ground) {
          ret->points.push_back(point_cloud->points[index]);
          ret->width++;
        }
      }
    }
  }

  // MISSING: Define a solution for the plane need in evaluator, himmelsbach does not estimate a
  // plane
  plane = Plane(0, 0, 1, -1.2);
}

void Himmelsbach::split_point_cloud(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                    std::vector<Slice>& splited_cloud,
                                    const SplitParameters split_params) const {
  int n_points_per_ring = static_cast<int>(
      std::ceil(split_params.angle_resolution / split_params.lidar_horizontal_resolution));
  int slice_idx = 0;
  int prev_ring = -1;
  int lines_for_next_slice = n_points_per_ring;
  splited_cloud.push_back(Slice());

  // For each point in the cloud assign to its corresponding slice and ring
  for (int i = 0; i < static_cast<int>(cloud->points.size()); i++) {
    const auto& pt = cloud->points[i];

    // Mark every point as non-ground initially
    splited_cloud[slice_idx].rings[pt.ring].indices.push_back(std::make_pair(i, false));
    int idx_in_ring = static_cast<int>(splited_cloud[slice_idx].rings[pt.ring].indices.size()) - 1;

    // Update min height index per ring
    if (splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx == -1) {
      splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx = idx_in_ring;
    } else if (pt.z <
               cloud->points[splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx].z) {
      splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx = idx_in_ring;
    }

    // If the current ring is smaller than the previous ring, we are in a new vertical line of
    // lidar points
    if (pt.ring < prev_ring) {
      lines_for_next_slice--;
    }

    // Processed all vertical lines of the current slice, go to the next slice
    if (lines_for_next_slice <= 0) {
      slice_idx++;
      splited_cloud.push_back(Slice());
      lines_for_next_slice = n_points_per_ring;
    }
    prev_ring = pt.ring;
  }
}

void Himmelsbach::process_slice(const pcl::PointCloud<PointXYZIR>::Ptr& cloud, Slice& slice,
                                const double ground_height_reference) const {
  bool first_ring = true;
  int previous_ground_point_index = -1;

  for (int i = NUM_RINGS - 1; i >= 0; i--) {
    if (slice.rings[i].indices.empty()) {
      continue;
    }

    // Idx in the indices vector of the lowest point in the ring
    int min_height_idx = slice.rings[i].indices_min_height_idx;

    // Point index in the original point cloud of the lowest point in the ring
    int min_height_pt_idx = slice.rings[i].indices[min_height_idx].first;

    if (first_ring) {
      // If the lowest point in the first ring is close enough to the ground height reference,
      // mark it as ground
      if (std::abs(cloud->points[min_height_pt_idx].z - ground_height_reference) < this->epsilon) {
        slice.rings[i].indices[min_height_idx].second = true;  // Mark as ground

        // Mark all points close enough to the lowest point as ground
        for (int j = 0; j < static_cast<int>(slice.rings[i].indices.size()); j++) {
          int idx = slice.rings[i].indices[j].first;
          if (std::abs(cloud->points[idx].z - cloud->points[min_height_pt_idx].z) < this->epsilon) {
            slice.rings[i].indices[j].second = true;  // Mark as ground
          }
        }

        previous_ground_point_index = min_height_pt_idx;
        first_ring = false;
      } else {
        continue;  // If not close enough, skip this ring
      }
    }

    // Store the lowest ground point in the ring
    int lowest_ground_idx_in_ring = -1;
    double lowest_ground_z_in_ring = std::numeric_limits<double>::max();

    // Reduce max allowed slope according to distance
    float distance =
        std::hypot(cloud->points[min_height_pt_idx].x, cloud->points[min_height_pt_idx].y);
    double adjusted_max_slope =
        std::max(this->max_slope - (this->slope_reduction * (distance / this->distance_reduction)),
                 this->min_slope);

    // For each point in the ring, calculate the slope to the previous ground point
    for (int j = 0; j < static_cast<int>(slice.rings[i].indices.size()); j++) {
      int idx = slice.rings[i].indices[j].first;
      double dz = cloud->points[idx].z - cloud->points[previous_ground_point_index].z;
      double dr = std::hypot(cloud->points[idx].x - cloud->points[previous_ground_point_index].x,
                             cloud->points[idx].y - cloud->points[previous_ground_point_index].y);

      if (dr < 1e-6) {
        continue;  // Skip points that are too close in xy distance because slope is not defined
      }

      double slope = std::abs(dz / dr);

      if (slope < adjusted_max_slope) {
        slice.rings[i].indices[j].second = true;  // Mark as ground

        // Update lowest ground point in the ring
        if (cloud->points[idx].z < lowest_ground_z_in_ring) {
          lowest_ground_z_in_ring = cloud->points[idx].z;
          lowest_ground_idx_in_ring = idx;
        }
      }
    }

    // Update previous ground point for next ring
    if (lowest_ground_idx_in_ring != -1) {
      previous_ground_point_index = lowest_ground_idx_in_ring;
    }
  }
}

double Himmelsbach::calculate_height_reference(const pcl::PointCloud<PointXYZIR>::Ptr& point_cloud,
                                               const std::vector<Slice>& splited_cloud,
                                               int cur_slice_idx) const {
  std::vector<int> slice_min_indices;

  // Compute range of slices to consider
  int half_window = this->adjacent_slices / 2;
  int start_idx = std::max(0, cur_slice_idx - half_window);
  int end_idx = std::min(static_cast<int>(splited_cloud.size()) - 1, cur_slice_idx + half_window);

  // Collect the min height indices for all slices in the window
  for (int j = start_idx; j <= end_idx; j++) {
    int min_idx = find_closest_ring_min_height_idx(splited_cloud[j]);
    slice_min_indices.push_back(min_idx);
  }

  // Calculate ground reference as the minimum z among valid slice indices
  float ground_height_reference = std::numeric_limits<float>::max();
  for (int idx : slice_min_indices) {
    if (idx != -1) {
      ground_height_reference = std::min(ground_height_reference, point_cloud->points[idx].z);
    }
  }

  return ground_height_reference;
}

int Himmelsbach::find_closest_ring_min_height_idx(const Slice& slice) const {
  int min_height_idx = -1;

  for (int i = NUM_RINGS - 1; i >= 0; i--) {
    if (slice.rings[i].indices.empty()) {
      continue;  // Skip empty rings
    }

    int idx_in_ring = slice.rings[i].indices_min_height_idx;
    min_height_idx = slice.rings[i].indices[idx_in_ring].first;
    break;
  }

  return min_height_idx;
}