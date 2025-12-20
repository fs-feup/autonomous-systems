#include "ground_removal/himmelsbach.hpp"

Himmelsbach::Himmelsbach(const double grid_angle, const double max_slope,
                         const double initial_alpha, const double alpha_augmentation_m,
                         const double start_augmentation, TrimmingParameters trim_params)
    : grid_angle_(grid_angle),
      max_slope_(max_slope),
      initial_alpha_(initial_alpha),
      alpha_augmentation_m_(alpha_augmentation_m),
      start_augmentation_(start_augmentation),
      trim_params_(trim_params) {
  // Preallocate slices
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

  // Resize output data to its final size
  ground_removed_point_cloud->data.resize(ground_removed_point_cloud->width *
                                          LidarPoint::POINT_STEP);
  ground_removed_point_cloud->row_step = ground_removed_point_cloud->width * LidarPoint::POINT_STEP;
}

void Himmelsbach::process_slice(
    const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud, size_t slice_idx,
    GroundGrid& ground_grid) const {
  // The first ground point is assumed to be directly below the car
  double previous_ground_point_x = 0.0;
  double previous_ground_point_y = 0.0;
  double previous_ground_point_z = -trim_params_.lidar_height;

  auto& cloud_data = trimmed_point_cloud->data;
  auto& output_data = ground_removed_point_cloud->data;

  // Indices of ground candidate points
  std::vector<int> ground_indices;
  std::vector<double> ground_distances;

  // For each ring in the slice, from highest to lowest, compare points to previous ground point
  for (int ring_idx = NUM_RINGS - 1; ring_idx >= 0; --ring_idx) {
    const auto& ring = slices_->at(slice_idx).rings[ring_idx];
    if (ring.indices.empty()) {
      continue;
    }

    int lowest_ground_idx_in_ring = -1;
    float previous_ground_point_distance =
        std::hypot(previous_ground_point_x, previous_ground_point_y);

    // Process each point in the ring comparing to the previous ground point
    for (int idx : ring.indices) {
      float pt_x = *reinterpret_cast<float*>(&cloud_data[LidarPoint::PointX(idx)]);
      float pt_y = *reinterpret_cast<float*>(&cloud_data[LidarPoint::PointY(idx)]);
      float pt_z = *reinterpret_cast<float*>(&cloud_data[LidarPoint::PointZ(idx)]);

      float current_ground_point_distance = std::hypot(pt_x, pt_y);

      if (current_ground_point_distance - previous_ground_point_distance <
              (-5 - (0.15 * ring_idx)) ||
          current_ground_point_distance - previous_ground_point_distance >
              (5 + (0.15 * ring_idx))) {
        // It is noise, ignore
        continue;
      } else if (current_ground_point_distance <= previous_ground_point_distance) {
        // Write it to the output cloud
        size_t write_idx = ground_removed_point_cloud->width;
        std::memcpy(&output_data[write_idx * LidarPoint::POINT_STEP],
                    &cloud_data[idx * LidarPoint::POINT_STEP], LidarPoint::POINT_STEP);
        ground_removed_point_cloud->width++;
      } else {
        // Compute slope to previous ground point
        float dz = pt_z - previous_ground_point_z;
        float dr = current_ground_point_distance - previous_ground_point_distance;
        if (dr < 0.001) {
          dr = 0.001;
        }

        float slope = std::abs(dz / dr);

        if (slope < this->max_slope_) {
          // Point is a ground candidate
          ground_indices.push_back(idx);
          ground_distances.push_back(current_ground_point_distance);
        } else {
          // Point is non-ground, write to output cloud
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * LidarPoint::POINT_STEP],
                      &cloud_data[idx * LidarPoint::POINT_STEP], LidarPoint::POINT_STEP);
          ground_removed_point_cloud->width++;
        }
      }
    }

    // Evaluate ground candidates to see if they are truly ground.
    // This is done by calculating the median distance of the farthest 10% of the points, and
    // checking the distance of the others to that reference. When the points is close to the ground
    // but closer to the car, it means it intercepted some object. If it did not intercept anything,
    // the point should be close to the reference. This helps to avoid including points that are
    // close to the ground but actually belong to obstacles (lower part of cones).
    if (!ground_indices.empty()) {
      int num_points = static_cast<int>(ground_distances.size());
      int top_k = std::max<int>(1, num_points / 10);

      // Partition so that the top 10% farthest values are at the end
      std::nth_element(ground_distances.begin(), ground_distances.end() - top_k,
                       ground_distances.end());

      // Calculate median of the top 10% farthest points
      std::vector<double> top_farthest(ground_distances.end() - top_k, ground_distances.end());
      std::sort(top_farthest.begin(), top_farthest.end());
      double r_ref = top_farthest[top_farthest.size() / 2];

      // Evaluate each ground candidate compared to the reference distance
      for (int idx : ground_indices) {
        float gx = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(idx)]);
        float gy = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(idx)]);
        float gz = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(idx)]);
        double r = std::hypot(gx, gy);

        double threshold = this->initial_alpha_;
        if (r > this->start_augmentation_) {
          threshold += this->alpha_augmentation_m_ * (r - this->start_augmentation_);
        }

        if (r < r_ref - threshold) {
          // too close to the car, non-ground
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * LidarPoint::POINT_STEP],
                      &cloud_data[idx * LidarPoint::POINT_STEP], LidarPoint::POINT_STEP);
          ground_removed_point_cloud->width++;
        } else {
          // Ground, update the ground grid and lowest ground point in the ring
          ground_grid.set_ground_height(gx, gy, gz);
          if (lowest_ground_idx_in_ring == -1 ||
              gz < *reinterpret_cast<const float*>(
                       &cloud_data[LidarPoint::PointZ(lowest_ground_idx_in_ring)])) {
            lowest_ground_idx_in_ring = idx;
          }
        }
      }

      if (lowest_ground_idx_in_ring != -1) {
        previous_ground_point_x =
            *reinterpret_cast<float*>(&cloud_data[LidarPoint::PointX(lowest_ground_idx_in_ring)]);
        previous_ground_point_y =
            *reinterpret_cast<float*>(&cloud_data[LidarPoint::PointY(lowest_ground_idx_in_ring)]);
        previous_ground_point_z =
            *reinterpret_cast<float*>(&cloud_data[LidarPoint::PointZ(lowest_ground_idx_in_ring)]);
      }

      ground_indices.clear();
      ground_distances.clear();
    }
  }
}

void Himmelsbach::split_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud) const {
  // Clear previous indices
  for (auto& slice : *slices_) {
    for (auto& ring : slice.rings) {
      ring.indices.clear();
    }
  }

  const size_t num_points = input_cloud->width * input_cloud->height;
  if (num_points == 0) return;

  const auto& cloud = input_cloud->data;
  double slice_angle_rad = grid_angle_ * (M_PI / 180.0);

  // Calculate the starting azimuth based on FOV
  double fov_rad = trim_params_.fov * (M_PI / 180.0);
  double min_azimuth = -(fov_rad / 2.0);

  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud[LidarPoint::PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud[LidarPoint::PointY(i)]);
    uint16_t ring = *reinterpret_cast<const uint16_t*>(&cloud[LidarPoint::PointRing(i)]);

    double azimuth = std::atan2(y, x);
    double relative_azimuth = azimuth - min_azimuth;

    if (relative_azimuth < 0 || relative_azimuth >= fov_rad) {
      continue;
    }

    int slice_idx = static_cast<int>(relative_azimuth / slice_angle_rad);

    if (slice_idx >= 0 && slice_idx < static_cast<int>(slices_->size())) {
      slices_->at(slice_idx).rings[ring].indices.push_back(i);
    }
  }
}