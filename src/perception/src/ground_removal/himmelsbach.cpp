#include "ground_removal/himmelsbach.hpp"

Himmelsbach::Himmelsbach(const double max_slope, const double epsilon, const int adjacent_slices)
    : max_slope(max_slope), epsilon(epsilon), adjacent_slices(adjacent_slices) {}

void Himmelsbach::ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                                 const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                                 [[maybe_unused]] const SplitParameters split_params) const {
  // Slip the point cloud into slices, each slice contains several rings
  std::vector<Slice> splited_cloud;
  split_point_cloud(point_cloud, splited_cloud, split_params);

  for (int i = 0; i < static_cast<int>(splited_cloud.size()); ++i) {
    float ground_height_reference = calculate_height_reference(point_cloud, splited_cloud, i);
    process_slice(point_cloud, splited_cloud[i], ground_height_reference);
  }

  ret->points.clear();
  ret->height = 1;
  ret->width = 0;
  ret->is_dense = false;

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

  plane = Plane(1, 1, 1, 1);
}

void Himmelsbach::split_point_cloud(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                    std::vector<Slice>& splited_cloud,
                                    const SplitParameters split_params) const {
  int n_slices =
      static_cast<int>(std::ceil(split_params.fov_angle / split_params.angle_resolution));
  int n_points_per_ring = static_cast<int>(
      std::ceil(split_params.angle_resolution / split_params.lidar_horizontal_resolution));
  int slice_idx = 0;
  int prev_ring = -1;
  int lines_for_next_slice = n_points_per_ring;
  int slice_count = 0;
  splited_cloud.push_back(Slice());

  // For each point in the cloud assign to its corresponding slice and ring
  for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
    const auto& pt = cloud->points[i];
    // Mark every point as non-ground initially
    splited_cloud[slice_idx].rings[pt.ring].indices.push_back(std::make_pair(i, false));
    slice_count++;

    int idx_in_ring = static_cast<int>(splited_cloud[slice_idx].rings[pt.ring].indices.size()) - 1;
    // Update min height index per ring
    if (splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx == -1) {
      splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx = idx_in_ring;
    } else if (pt.z <
               cloud->points[splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx].z) {
      splited_cloud[slice_idx].rings[pt.ring].indices_min_height_idx = idx_in_ring;
    }

    // If the current ring if smaller than the previous ring, we are in a new vertical line of
    // lidar points
    if (pt.ring < prev_ring) {
      lines_for_next_slice--;
    }

    // Processed all vertical lines of the current slice, go to the next slice
    if (lines_for_next_slice <= 0) {
      slice_idx++;
      splited_cloud.push_back(Slice());
      slice_count = 0;
      lines_for_next_slice = n_points_per_ring;
      if (slice_idx >= n_slices) {
        RCLCPP_ERROR(rclcpp::get_logger("Himmelsbach"),
                     "Still missing points and all slices were occupied");
        break;  // stop if done
      }
    }
    prev_ring = pt.ring;
  }
}

void Himmelsbach::process_slice(const pcl::PointCloud<PointXYZIR>::Ptr& cloud, Slice& slice,
                                const float ground_height_reference) const {
  bool first_ring = true;
  long previous_ground_point_index = -1;
  for (int i = NUM_RINGS - 1; i >= 0; i--) {
    if (slice.rings[i].indices.empty()) {
      continue;  // Skip empty rings
    }

    // Idx in the indices vector of the lowest point in the ring
    int min_height_idx = slice.rings[i].indices_min_height_idx;
    // Point index in the original point cloud of the lowest point in the ring
    long min_height_pt_idx = slice.rings[i].indices[min_height_idx].first;

    if (first_ring) {
      // If the lowest point in the first ring is close enough to the ground height reference,
      // mark it as ground
      if (std::abs(cloud->points[min_height_pt_idx].z - ground_height_reference) < this->epsilon) {
        slice.rings[i].indices[min_height_idx].second = true;  // Mark as ground

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

    long lowest_ground_idx_in_ring = -1;
    float lowest_ground_z_in_ring = std::numeric_limits<float>::max();

    for (int j = 0; j < static_cast<int>(slice.rings[i].indices.size()); j++) {
      long idx = slice.rings[i].indices[j].first;
      float dz = cloud->points[idx].z - cloud->points[previous_ground_point_index].z;
      float dr = std::hypot(cloud->points[idx].x - cloud->points[previous_ground_point_index].x,
                            cloud->points[idx].y - cloud->points[previous_ground_point_index].y);

      float slope = (dr > 1e-6f) ? std::abs(dz / dr) : 0.0f;

      if (slope < this->max_slope) {
        slice.rings[i].indices[j].second = true;

        // Track lowest ground point in this ring for next reference
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

float Himmelsbach::calculate_height_reference(const pcl::PointCloud<PointXYZIR>::Ptr& point_cloud,
                                              const std::vector<Slice>& splited_cloud,
                                              int cur_slice_idx) const {
  std::vector<int> slice_min_indices;

  // Compute range of slices to consider
  int half_window = this->adjacent_slices / 2;
  int start_idx = std::max(0, cur_slice_idx - half_window);
  int end_idx = std::min(static_cast<int>(splited_cloud.size()) - 1, cur_slice_idx + half_window);

  // Collect the min height indices for all slices in the window
  for (int j = start_idx; j <= end_idx; ++j) {
    int min_idx = find_closest_ring_min_height_idx(point_cloud, splited_cloud[j]);
    slice_min_indices.push_back(min_idx);
  }

  // Calculate ground reference as the minimum z among valid slice indices
  float ground_height_reference = std::numeric_limits<float>::max();
  for (int idx : slice_min_indices) {
    if (idx != -1) {
      ground_height_reference = std::min(ground_height_reference, point_cloud->points[idx].z);
    }
  }

  // Optional: fallback if no valid indices
  if (ground_height_reference == std::numeric_limits<float>::max()) {
    ground_height_reference = 0.0f;
  }
  return ground_height_reference;
}

int Himmelsbach::find_closest_ring_min_height_idx(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                                  const Slice& slice) const {
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