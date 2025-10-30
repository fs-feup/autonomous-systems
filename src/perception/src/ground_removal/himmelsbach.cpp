#include "ground_removal/himmelsbach.hpp"

#include "rclcpp/rclcpp.hpp"

Himmelsbach::Himmelsbach(const double max_slope, const double epsilon, const int adjacent_slices,
                         const double slope_reduction, const double distance_reduction,
                         const double min_slope)
    : max_slope(max_slope),
      epsilon(epsilon),
      adjacent_slices(adjacent_slices),
      slope_reduction(slope_reduction),
      distance_reduction(distance_reduction),
      min_slope(min_slope) {}

void Himmelsbach::ground_removal(
    const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud, Plane& plane,
    SplitParameters& split_params) const {
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

  // Split the cloud into slices
  std::vector<Slice> slices;
  split_point_cloud(trimmed_point_cloud, slices, split_params);

  // Process each slice
  for (size_t slice_idx = 0; slice_idx < slices.size(); ++slice_idx) {
    process_slice(trimmed_point_cloud, ground_removed_point_cloud, slices, slice_idx);
  }
  ground_removed_point_cloud->data.resize(ground_removed_point_cloud->width *
                                          ground_removed_point_cloud->point_step);
  ground_removed_point_cloud->row_step =
      ground_removed_point_cloud->width * ground_removed_point_cloud->point_step;

  plane = Plane(0, 0, 1, -1.2);
}

void Himmelsbach::process_slice(
    const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_point_cloud,
    const std::vector<Slice>& slices, size_t slice_idx) const {
  // Array pointers for point cloud data
  const uint8_t* cloud_data = trimmed_point_cloud->data.data();
  uint8_t* output_data = ground_removed_point_cloud->data.data();

  double ground_height_reference =
      calculate_height_reference(trimmed_point_cloud, slices, slice_idx);

  bool first_ring = true;
  double previous_ground_point_x = 0;
  double previous_ground_point_y = 0;
  double previous_ground_point_z = 0;

  for (int ring_idx = NUM_RINGS - 1; ring_idx >= 0; --ring_idx) {
    const auto& ring = slices[slice_idx].rings[ring_idx];
    if (ring.indices.empty()) continue;

    int min_idx = ring.min_height_idx;
    double ground_z_accumulator = 0.0;
    int ground_point_count = 0;
    int lowest_ground_idx_in_ring = -1;

    if (first_ring) {
      for (int idx : ring.indices) {
        float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint(idx).z()]);
        if (std::abs(z - ground_height_reference) < epsilon) {
          // Only mark as not first ring after finding the first ground point

          if (lowest_ground_idx_in_ring == -1 ||
              z < *reinterpret_cast<const float*>(
                      &cloud_data[LidarPoint(lowest_ground_idx_in_ring).z()])) {
            lowest_ground_idx_in_ring = idx;
          }

          ground_z_accumulator += z;
          ground_point_count++;
          first_ring = false;
        } else {
          size_t write_idx = ground_removed_point_cloud->width;
          std::memcpy(&output_data[write_idx * trimmed_point_cloud->point_step],
                      &cloud_data[idx * trimmed_point_cloud->point_step],
                      trimmed_point_cloud->point_step);
          ground_removed_point_cloud->width++;
        }
      }

      if (ground_point_count > 0) {
        previous_ground_point_x =
            *reinterpret_cast<const float*>(&cloud_data[LidarPoint(lowest_ground_idx_in_ring).x()]);
        previous_ground_point_y =
            *reinterpret_cast<const float*>(&cloud_data[LidarPoint(lowest_ground_idx_in_ring).y()]);
        previous_ground_point_z = ground_z_accumulator / ground_point_count;
      }
      continue;
    }

    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint(min_idx).x()]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint(min_idx).y()]);
    float distance = std::hypot(x, y);
    double adjusted_max_slope =
        std::max(max_slope - (slope_reduction * (distance / distance_reduction)), min_slope);

    for (int idx : ring.indices) {
      float pt_x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint(idx).x()]);
      float pt_y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint(idx).y()]);
      float pt_z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint(idx).z()]);

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
            pt_z < *reinterpret_cast<const float*>(
                       &cloud_data[LidarPoint(lowest_ground_idx_in_ring).z()])) {
          lowest_ground_idx_in_ring = idx;
        }
      } else {
        size_t write_idx = ground_removed_point_cloud->width;
        std::memcpy(&output_data[write_idx * ground_removed_point_cloud->point_step],
                    &cloud_data[idx * ground_removed_point_cloud->point_step],
                    ground_removed_point_cloud->point_step);
        ground_removed_point_cloud->width++;
      }
    }

    if (ground_point_count > 0) {
      previous_ground_point_x =
          *reinterpret_cast<const float*>(&cloud_data[LidarPoint(lowest_ground_idx_in_ring).x()]);
      previous_ground_point_y =
          *reinterpret_cast<const float*>(&cloud_data[LidarPoint(lowest_ground_idx_in_ring).y()]);
      previous_ground_point_z = ground_z_accumulator / ground_point_count;
    }
  }
}

void Himmelsbach::split_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
                                    std::vector<Slice>& slices,
                                    const SplitParameters& split_params) const {
  const uint8_t* cloud_data = cloud->data.data();
  int n_points_per_ring = static_cast<int>(
      std::ceil(split_params.angle_resolution / split_params.lidar_horizontal_resolution));

  int slice_idx = 0;
  int prev_ring = -1;
  int lines_for_next_slice = n_points_per_ring;
  slices.push_back(Slice());

  const size_t num_points = cloud->width * cloud->height;

  for (size_t i = 0; i < num_points; ++i) {
    LidarPoint pt(i);
    uint16_t ring = *reinterpret_cast<const uint16_t*>(&cloud_data[pt.ring()]);

    auto& ring_struct = slices[slice_idx].rings[ring];
    ring_struct.indices.push_back(i);

    if (ring_struct.min_height_idx == -1) {
      ring_struct.min_height_idx = i;
    } else {
      float z = *reinterpret_cast<const float*>(&cloud_data[pt.z()]);
      float min_z =
          *reinterpret_cast<const float*>(&cloud_data[LidarPoint(ring_struct.min_height_idx).z()]);
      if (z < min_z) ring_struct.min_height_idx = i;
    }

    if (prev_ring != -1 && ring < prev_ring) {
      lines_for_next_slice--;
    }

    if (lines_for_next_slice <= 0) {
      slice_idx++;
      slices.push_back(Slice());
      lines_for_next_slice = n_points_per_ring;
    }

    prev_ring = ring;
  }
}

double Himmelsbach::calculate_height_reference(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, const std::vector<Slice>& slices,
    int slice_idx) const {
  int half_window = adjacent_slices / 2;
  int start_idx = std::max(0, slice_idx - half_window);
  int end_idx = std::min(static_cast<int>(slices.size()) - 1, slice_idx + half_window);

  const uint8_t* cloud_data = cloud->data.data();

  std::vector<float> z_values;
  for (int i = start_idx; i <= end_idx; ++i) {
    int min_idx = find_closest_ring_min_height(slices[i]);
    if (min_idx != -1) {
      float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint(min_idx).z()]);
      z_values.push_back(z);
    }
  }

  if (z_values.empty()) return 0.0f;

  std::sort(z_values.begin(), z_values.end());
  float median_z = z_values[z_values.size() / 2];
  return median_z;
}

int Himmelsbach::find_closest_ring_min_height(const Slice& slice) const {
  for (int i = NUM_RINGS - 1; i >= 0; --i) {
    if (!slice.rings[i].indices.empty()) {
      return slice.rings[i].min_height_idx;
    }
  }
  return -1;
}
