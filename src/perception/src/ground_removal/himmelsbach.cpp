#include "ground_removal/himmelsbach.hpp"

#include "rclcpp/rclcpp.hpp"
Himmelsbach::Himmelsbach(const int ground_reference_slices, const double epsilon,
                         const double max_slope, const double min_slope,
                         const double slope_reduction_m, const double start_reduction,
                         const double initial_alpha, const double alpha_augmentation_m,
                         const double start_augmentation, SplitParameters split_params)
    : ground_reference_slices_(ground_reference_slices),
      epsilon_(epsilon),
      max_slope_(max_slope),
      min_slope_(min_slope),
      slope_reduction_m_(slope_reduction_m),
      start_reduction_(start_reduction),
      initial_alpha_(initial_alpha),
      alpha_augmentation_m_(alpha_augmentation_m),
      start_augmentation_(start_augmentation),
      split_params_(split_params) {
  // --- Preallocate slices ---
  const int num_slices =
      static_cast<int>(std::ceil(split_params_.fov / split_params_.angle_resolution));
  slices_ = std::make_shared<std::vector<Slice>>(num_slices);
}

void Himmelsbach::ground_removal(
    const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud, Plane& plane) const {
  // Initialize output cloud
  ground_removed_point_cloud->header = trimmed_point_cloud->header;
  ground_removed_point_cloud->height = 1;
  ground_removed_point_cloud->is_bigendian = trimmed_point_cloud->is_bigendian;
  ground_removed_point_cloud->is_dense = true;
  ground_removed_point_cloud->point_step = trimmed_point_cloud->point_step;
  ground_removed_point_cloud->row_step = 0;
  ground_removed_point_cloud->fields = trimmed_point_cloud->fields;
  ground_removed_point_cloud->width = 0;
  ground_removed_point_cloud->data.resize(trimmed_point_cloud->width *
                                          trimmed_point_cloud->point_step);

  split_point_cloud(trimmed_point_cloud);

  // Process each slice
  for (size_t slice_idx = 0; slice_idx < slices_->size(); ++slice_idx) {
    process_slice(trimmed_point_cloud, ground_removed_point_cloud, slice_idx);
  }

  ground_removed_point_cloud->data.resize(ground_removed_point_cloud->width * POINT_STEP);
  ground_removed_point_cloud->row_step = ground_removed_point_cloud->width * POINT_STEP;

  plane = Plane(0, 0, 1, -1.2);
}

void Himmelsbach::process_slice(
    const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud, size_t slice_idx) const {
  // The first ground point is assumed to be directly below the LIDAR at lidar height
  double previous_ground_point_x = 0.0;
  double previous_ground_point_y = 0.0;
  double previous_ground_point_z = -split_params_.lidar_height;

  auto& cloud_data = trimmed_point_cloud->data;
  auto& output_data = ground_removed_point_cloud->data;
  bool first_ring = true;
  const double ground_height_reference = calculate_ground_reference(trimmed_point_cloud, slice_idx);

  std::vector<int> ground_indices;

  for (int ring_idx = NUM_RINGS - 1; ring_idx >= 0; --ring_idx) {
    const auto& ring = slices_->at(slice_idx).rings[ring_idx];
    if (ring.indices.empty()) continue;

    int lowest_ground_idx_in_ring = -1;

    if (first_ring) {
      for (int idx : ring.indices) {
        float z = *reinterpret_cast<const float*>(&cloud_data[PointZ(idx)]);
        if (std::abs(z - ground_height_reference) < this->epsilon_) {
          // Only mark as not first ring after finding the first ground point

          if (lowest_ground_idx_in_ring == -1 ||
              z < *reinterpret_cast<const float*>(&cloud_data[PointZ(lowest_ground_idx_in_ring)])) {
            lowest_ground_idx_in_ring = idx;
          }

          first_ring = false;
        } else {
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                      POINT_STEP);
          ground_removed_point_cloud->width++;
        }
      }

      if (lowest_ground_idx_in_ring != -1) {
        previous_ground_point_x =
            *reinterpret_cast<const float*>(&cloud_data[PointX(lowest_ground_idx_in_ring)]);
        previous_ground_point_y =
            *reinterpret_cast<const float*>(&cloud_data[PointY(lowest_ground_idx_in_ring)]);
        previous_ground_point_z =
            *reinterpret_cast<const float*>(&cloud_data[PointZ(lowest_ground_idx_in_ring)]);
      }
      continue;
    }

    // Use the first point in the ring to calculate distance for slope adjustment
    float x = *reinterpret_cast<float*>(&cloud_data[PointX(ring.indices[0])]);
    float y = *reinterpret_cast<float*>(&cloud_data[PointY(ring.indices[0])]);
    float distance = std::hypot(x, y);
    double adjusted_max_slope = this->max_slope_;
    if (distance > this->start_reduction_) {
      adjusted_max_slope -=
          this->slope_reduction_m_ * (static_cast<double>(distance - this->start_reduction_));
      adjusted_max_slope = std::max(adjusted_max_slope, this->min_slope_);
    }

    for (int idx : ring.indices) {
      float pt_x = *reinterpret_cast<float*>(&cloud_data[PointX(idx)]);
      float pt_y = *reinterpret_cast<float*>(&cloud_data[PointY(idx)]);
      float pt_z = *reinterpret_cast<float*>(&cloud_data[PointZ(idx)]);

      double dz = pt_z - previous_ground_point_z;

      if (std::hypot(pt_x, pt_y) < (std::hypot(previous_ground_point_x, previous_ground_point_y))) {
        // If point is closer than the previous ground point, it cannot be ground
        size_t write_idx = ground_removed_point_cloud->width;
        std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                    POINT_STEP);
        ground_removed_point_cloud->width++;
      } else {
        double dr = std::hypot(pt_x - previous_ground_point_x, pt_y - previous_ground_point_y);
        if (dr < 0.001) {
          dr = 0.001;
        }

        double slope = std::abs(dz / dr);

        if (slope < adjusted_max_slope) {
          // Point is ground
          ground_indices.push_back(idx);
        } else {
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                      POINT_STEP);
          ground_removed_point_cloud->width++;
        }
      }
    }  // end for each idx

    if (!ground_indices.empty()) {
      // --- Find the farthest ground candidate ---
      double max_distance = 0.0;
      for (int idx : ground_indices) {
        float gx = *reinterpret_cast<const float*>(&cloud_data[PointX(idx)]);
        float gy = *reinterpret_cast<const float*>(&cloud_data[PointY(idx)]);
        double dist = std::hypot(gx, gy);
        if (dist > max_distance) {
          max_distance = dist;
        }
      }

      for (int idx : ground_indices) {
        float gx = *reinterpret_cast<const float*>(&cloud_data[PointX(idx)]);
        float gy = *reinterpret_cast<const float*>(&cloud_data[PointY(idx)]);
        float gz = *reinterpret_cast<const float*>(&cloud_data[PointZ(idx)]);
        double dist = std::hypot(gx, gy);

        double alpha = this->initial_alpha_;
        if (dist > this->start_augmentation_) {
          alpha += this->alpha_augmentation_m_ * (dist - this->start_augmentation_);
        }

        if (max_distance - dist > alpha) {
          // Too far from the farthest ground point in this ring, classify as non-ground
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                      POINT_STEP);
          ground_removed_point_cloud->width++;
        } else {
          if (lowest_ground_idx_in_ring == -1 ||
              gz <
                  *reinterpret_cast<const float*>(&cloud_data[PointZ(lowest_ground_idx_in_ring)])) {
            lowest_ground_idx_in_ring = idx;
          }
        }
      }
    }

    if (lowest_ground_idx_in_ring != -1) {
      previous_ground_point_x =
          *reinterpret_cast<float*>(&cloud_data[PointX(lowest_ground_idx_in_ring)]);
      previous_ground_point_y =
          *reinterpret_cast<float*>(&cloud_data[PointY(lowest_ground_idx_in_ring)]);
      previous_ground_point_z =
          *reinterpret_cast<float*>(&cloud_data[PointZ(lowest_ground_idx_in_ring)]);
    }

    ground_indices.clear();
  }
}

void Himmelsbach::split_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud) const {
  // Clear existing indices
  for (auto& slice : *slices_) {
    for (auto& ring : slice.rings) {
      ring.indices.clear();
    }
  }

  const size_t num_points = input_cloud->width * input_cloud->height;
  if (num_points == 0) return;

  const auto& cloud = input_cloud->data;
  const int num_slices = slices_->size();

  // Each slice covers a fixed azimuth range
  const double slice_angle = 2.0 * M_PI / static_cast<double>(num_slices);

  for (size_t i = 0; i < num_points; ++i) {
    // Extract XYZ and ring from point cloud
    float x = *reinterpret_cast<const float*>(&cloud[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud[PointY(i)]);
    uint16_t ring = *reinterpret_cast<const uint16_t*>(&cloud[PointRing(i)]);

    // Compute azimuth angle [0, 2π)
    double azimuth = std::atan2(y, x);
    if (azimuth < 0.0) azimuth += 2.0 * M_PI;

    if (azimuth <= 0 || azimuth >= 2 * M_PI) {
      // Ignore points behind the LiDAR
      continue;
    }

    // Determine slice index
    int slice_idx = static_cast<int>(azimuth / slice_angle);

    // Add point index to corresponding slice & ring
    auto& ring_struct = slices_->at(slice_idx).rings[ring];
    ring_struct.indices.push_back(i);
  }
}

double Himmelsbach::calculate_ground_reference(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud, int slice_idx) const {
  int half_window = this->ground_reference_slices_ / 2;
  int start_idx = std::max(0, slice_idx - half_window);
  int end_idx = std::min(static_cast<int>(slices_->size()) - 1, slice_idx + half_window);

  std::vector<float> z_values;
  for (int i = start_idx; i <= end_idx; ++i) {
    float min_z = find_closest_ring_min_height(input_cloud, i);

    if (min_z != std::numeric_limits<float>::max()) {
      z_values.push_back(min_z);
    }
  }

  if (z_values.empty()) return 0.0f;

  std::sort(z_values.begin(), z_values.end());
  float median_z = z_values[z_values.size() / 2];
  return median_z;
}

float Himmelsbach::find_closest_ring_min_height(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud, int slice_idx) const {
  for (int i = NUM_RINGS - 1; i >= 0; --i) {
    if (!slices_->at(slice_idx).rings[i].indices.empty()) {
      float min_height = std::numeric_limits<float>::max();
      for (int idx : slices_->at(slice_idx).rings[i].indices) {
        float z = *reinterpret_cast<const float*>(&input_cloud->data[PointZ(idx)]);
        if (z < min_height) {
          min_height = z;
        }
      }
      return min_height;
    }
  }
  return std::numeric_limits<float>::max();
}