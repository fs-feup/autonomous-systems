#include "fov_trimming/acceleration_trimming.hpp"

AccelerationTrimming::AccelerationTrimming(const TrimmingParameters params) { params_ = params; }

void AccelerationTrimming::fov_trimming(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_cloud) const {
  // Copy header and field layout
  trimmed_cloud->header = cloud->header;
  trimmed_cloud->height = 1;
  trimmed_cloud->is_dense = false;
  trimmed_cloud->fields = cloud->fields;
  trimmed_cloud->point_step = cloud->point_step;
  trimmed_cloud->row_step = 0;
  trimmed_cloud->width = 0;
  trimmed_cloud->data.resize(cloud->data.size());

  auto& cloud_data = cloud->data;
  size_t num_points = cloud->width * cloud->height;

  const double theta = params_.apply_rotation ? params_.rotation * M_PI / 180.0 : 0.0;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[PointY(i)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[PointZ(i)]);

    if (x == 0.0 && y == 0.0 && z == 0.0) {
      continue;
    }

    if (params_.apply_rotation) {
      const float rotated_x = static_cast<float>(x * cos_theta - y * sin_theta);
      const float rotated_y = static_cast<float>(x * sin_theta + y * cos_theta);
      x = rotated_x;
      y = rotated_y;
    }

    if (y < params_.acc_max_y && y > -params_.acc_max_y &&
        within_limits(x, y, z, params_, params_.acc_max_range)) {
      uint8_t* out = &trimmed_cloud->data[trimmed_cloud->width * POINT_STEP];
      std::memcpy(out, &cloud_data[PointX(i)], POINT_STEP);

      if (params_.apply_rotation) {
        *reinterpret_cast<float*>(out + PointX(0)) = x;
        *reinterpret_cast<float*>(out + PointY(0)) = y;
      }
      trimmed_cloud->width++;
    }
  }

  trimmed_cloud->data.resize(trimmed_cloud->width * POINT_STEP);  // Shrink to actual size
  trimmed_cloud->row_step = trimmed_cloud->width * POINT_STEP;
}
