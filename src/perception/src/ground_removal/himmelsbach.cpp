#include "ground_removal/himmelsbach.hpp"

#include "rclcpp/rclcpp.hpp"
Himmelsbach::Himmelsbach(const double grid_angle, const double max_slope, const double min_slope,
                         const double slope_reduction_m, const double start_reduction,
                         const double initial_alpha, const double alpha_augmentation_m,
                         const double start_augmentation, TrimmingParameters trim_params)
    : grid_angle_(grid_angle),
      max_slope_(max_slope),
      min_slope_(min_slope),
      slope_reduction_m_(slope_reduction_m),
      start_reduction_(start_reduction),
      initial_alpha_(initial_alpha),
      alpha_augmentation_m_(alpha_augmentation_m),
      start_augmentation_(start_augmentation),
      trim_params_(trim_params) {
  // --- Preallocate slices ---
  const int num_slices = static_cast<int>(std::ceil(trim_params.fov / grid_angle));
  slices_ = std::make_shared<std::vector<Slice>>(num_slices);
}

void Himmelsbach::ground_removal(
    const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud,
    GroundGrid& ground_grid) const {
  // Reset ground grid
  ground_grid.reset_grid();

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

  // Split point cloud into slices and rings
  split_point_cloud(trimmed_point_cloud);

  // Process each slice
  for (size_t slice_idx = 0; slice_idx < slices_->size(); ++slice_idx) {
    process_slice(trimmed_point_cloud, ground_removed_point_cloud, slice_idx, ground_grid);
  }

  ground_removed_point_cloud->data.resize(ground_removed_point_cloud->width * POINT_STEP);
  ground_removed_point_cloud->row_step = ground_removed_point_cloud->width * POINT_STEP;
}

void Himmelsbach::process_slice(
    const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud, size_t slice_idx,
    GroundGrid& ground_grid) const {
  // The first ground point is assumed to be directly below the LIDAR at lidar height
  double previous_ground_point_x = 0.0;
  double previous_ground_point_y = 0.0;
  double previous_ground_point_z = -trim_params_.lidar_height;

  auto& cloud_data = trimmed_point_cloud->data;
  auto& output_data = ground_removed_point_cloud->data;

  std::vector<int> ground_indices;

  for (int ring_idx = NUM_RINGS - 1; ring_idx >= 0; --ring_idx) {
    const auto& ring = slices_->at(slice_idx).rings[ring_idx];
    if (ring.indices.empty()) continue;

    int lowest_ground_idx_in_ring = -1;

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

    float previous_ground_point_distance =
        std::hypot(previous_ground_point_x, previous_ground_point_y);

    for (int idx : ring.indices) {
      float pt_x = *reinterpret_cast<float*>(&cloud_data[PointX(idx)]);
      float pt_y = *reinterpret_cast<float*>(&cloud_data[PointY(idx)]);
      float pt_z = *reinterpret_cast<float*>(&cloud_data[PointZ(idx)]);

      float dz = pt_z - previous_ground_point_z;
      float current_ground_point_distance = std::hypot(pt_x, pt_y);

      if (current_ground_point_distance < previous_ground_point_distance) {
        // If point is closer than the previous ground point, it cannot be ground
        size_t write_idx = ground_removed_point_cloud->width;
        std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                    POINT_STEP);
        ground_removed_point_cloud->width++;
      } else {
        float dr = current_ground_point_distance - previous_ground_point_distance;
        if (dr < 0.001) {
          dr = 0.001;
        }

        float slope = std::abs(dz / dr);

        if (slope < adjusted_max_slope) {
          // Point is at ground level
          ground_indices.push_back(idx);
        } else {
          // Point is non-ground
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                      POINT_STEP);
          ground_removed_point_cloud->width++;
        }
      }
    }  // end for each idx

    if (!ground_indices.empty()) {
      std::vector<double> r_values;
      std::vector<float> z_values;
      r_values.reserve(ground_indices.size());
      z_values.reserve(ground_indices.size());

      // --- Collect r and z for all ground candidates ---
      for (int idx : ground_indices) {
        float gx = *reinterpret_cast<const float*>(&cloud_data[PointX(idx)]);
        float gy = *reinterpret_cast<const float*>(&cloud_data[PointY(idx)]);
        float gz = *reinterpret_cast<const float*>(&cloud_data[PointZ(idx)]);
        double r = std::hypot(gx, gy);
        r_values.push_back(r);
        z_values.push_back(gz);
      }

      // --- Sort indices by r (not values themselves) ---
      std::vector<size_t> sorted_idx(r_values.size());
      std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
      std::sort(sorted_idx.begin(), sorted_idx.end(),
                [&](size_t a, size_t b) { return r_values[a] < r_values[b]; });

      // --- Take top 10% farthest points for regression ---
      size_t N = sorted_idx.size();
      size_t start_i = static_cast<size_t>(std::max<size_t>(0, N * 0.9));

      double sum_r = 0.0, sum_z = 0.0, sum_rz = 0.0, sum_r2 = 0.0;
      for (size_t i = start_i; i < N; ++i) {
        double r = r_values[sorted_idx[i]];
        double z = z_values[sorted_idx[i]];
        sum_r += r;
        sum_z += z;
        sum_rz += r * z;
        sum_r2 += r * r;
      }

      double n = N - start_i;
      double denom = n * sum_r2 - sum_r * sum_r;
      double a = 0.0, b = 0.0;
      if (denom > 1e-6) {
        a = (n * sum_rz - sum_r * sum_z) / denom;
        b = (sum_z - a * sum_r) / n;
      }

      // --- Use regression to classify ground/non-ground ---
      for (int idx : ground_indices) {
        float gx = *reinterpret_cast<const float*>(&cloud_data[PointX(idx)]);
        float gy = *reinterpret_cast<const float*>(&cloud_data[PointY(idx)]);
        float gz = *reinterpret_cast<const float*>(&cloud_data[PointZ(idx)]);
        double r = std::hypot(gx, gy);

        double expected_z = a * r + b;
        double dz = gz - expected_z;

        double alpha = this->initial_alpha_;
        if (r > this->start_augmentation_) {
          alpha += this->alpha_augmentation_m_ * (r - this->start_augmentation_);
        }

        if (dz > alpha) {
          // too high -> non-ground
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                      POINT_STEP);
          ground_removed_point_cloud->width++;
        } else {
          // ground
          ground_grid.set_ground_height(gx, gy, gz);
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
  const double slice_angle = M_PI / static_cast<double>(num_slices);

  for (size_t i = 0; i < num_points; ++i) {
    // Extract XYZ and ring from point cloud
    float x = *reinterpret_cast<const float*>(&cloud[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud[PointY(i)]);
    uint16_t ring = *reinterpret_cast<const uint16_t*>(&cloud[PointRing(i)]);

    // Compute azimuth angle [0, π), without driver correction
    // double azimuth = std::atan2(x, -y);

    // Compute azimuth angle [0, π), with driver correction
    double azimuth = std::atan2(y, x);
    if (azimuth < -M_PI_2 || azimuth > M_PI_2) continue;

    // Shift range from [-π/2, +π/2] → [0, π]
    azimuth += M_PI_2;

    // Determine slice index
    int slice_idx = static_cast<int>(azimuth / slice_angle);

    // Add point index to corresponding slice & ring
    auto& ring_struct = slices_->at(slice_idx).rings[ring];
    ring_struct.indices.push_back(i);
  }
}