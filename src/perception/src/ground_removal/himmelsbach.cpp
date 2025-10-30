#include "ground_removal/himmelsbach.hpp"

#include "rclcpp/rclcpp.hpp"

Himmelsbach::Himmelsbach(const double max_slope, const double epsilon, const double slope_reduction,
                         const double distance_reduction, const double min_slope,
                         SplitParameters split_params)
    : max_slope(max_slope),
      epsilon(epsilon),
      slope_reduction(slope_reduction),
      distance_reduction(distance_reduction),
      min_slope(min_slope),
      split_params(split_params) {
  const int num_slices =
      static_cast<int>(std::ceil(split_params.fov / split_params.angle_resolution));

  max_points_per_ring = static_cast<int>(
      std::ceil(split_params.angle_resolution / split_params.lidar_horizontal_resolution));

  // --- Preallocate slices ---
  slices_ = std::make_shared<std::vector<Slice>>(num_slices);
  for (auto& slice : *slices_) {
    for (auto& ring : slice.rings) {
      ring.indices.resize(max_points_per_ring);
    }
  }
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
  double previous_ground_point_z = -split_params.lidar_height;

  auto& cloud_data = trimmed_point_cloud->data;
  auto& output_data = ground_removed_point_cloud->data;

  for (int ring_idx = NUM_RINGS - 1; ring_idx >= 0; --ring_idx) {
    const auto& ring = slices_->at(slice_idx).rings[ring_idx];
    if (ring.indices.empty()) continue;

    double ground_z_accumulator = 0.0;
    int ground_point_count = 0;
    int lowest_ground_idx_in_ring = -1;

    // Use the first point in the ring to calculate distance for slope adjustment
    float x = *reinterpret_cast<float*>(&cloud_data[PointX(ring.indices[0])]);
    float y = *reinterpret_cast<float*>(&cloud_data[PointY(ring.indices[0])]);
    float distance = std::hypot(x, y);
    double adjusted_max_slope =
        std::max(max_slope - (slope_reduction * (distance / distance_reduction)), min_slope);

    for (int idx : ring.indices) {
      float pt_x = *reinterpret_cast<float*>(&cloud_data[PointX(idx)]);
      float pt_y = *reinterpret_cast<float*>(&cloud_data[PointY(idx)]);
      float pt_z = *reinterpret_cast<float*>(&cloud_data[PointZ(idx)]);

      double dz = pt_z - previous_ground_point_z;
      double dr = std::hypot(pt_x - previous_ground_point_x, pt_y - previous_ground_point_y);
      if (dr < 0.01) {
        dr = 0.01;
      }

      double slope = std::abs(dz / dr);

      if (slope < adjusted_max_slope) {
        // Point is ground
        ground_z_accumulator += pt_z;
        ground_point_count++;
        if (lowest_ground_idx_in_ring == -1 ||
            pt_z < *reinterpret_cast<float*>(&cloud_data[PointZ(lowest_ground_idx_in_ring)])) {
          lowest_ground_idx_in_ring = idx;
        }
      } else {
        size_t write_idx = ground_removed_point_cloud->width;
        std::memcpy(&output_data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                    POINT_STEP);
        ground_removed_point_cloud->width++;
      }
    }

    if (ground_point_count > 0) {
      previous_ground_point_x =
          *reinterpret_cast<float*>(&cloud_data[PointX(lowest_ground_idx_in_ring)]);
      previous_ground_point_y =
          *reinterpret_cast<float*>(&cloud_data[PointY(lowest_ground_idx_in_ring)]);
      previous_ground_point_z = ground_z_accumulator / ground_point_count;
    }
  }
}

void Himmelsbach::split_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud) const {
  for (auto& slice : *slices_) {
    for (auto& ring : slice.rings) {
      ring.indices.clear();
    }
  }

  int slice_idx = 0;
  int prev_ring = -1;
  int lines_for_next_slice = max_points_per_ring;
  const size_t num_points = input_cloud->width * input_cloud->height;
  auto& cloud = input_cloud->data;

  for (size_t i = 0; i < num_points; ++i) {
    uint16_t ring = *reinterpret_cast<uint16_t*>(&cloud[PointRing(i)]);

    auto& ring_struct = slices_->at(slice_idx).rings[ring];
    ring_struct.indices.push_back(i);

    if (prev_ring != -1 && ring < prev_ring) {
      lines_for_next_slice--;
    }

    if (lines_for_next_slice <= 0) {
      slice_idx++;
      lines_for_next_slice = max_points_per_ring;
    }

    prev_ring = ring;
  }
}