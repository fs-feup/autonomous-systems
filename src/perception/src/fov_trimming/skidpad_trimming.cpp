#include "fov_trimming/skidpad_trimming.hpp"

SkidpadTrimming::SkidpadTrimming(const TrimmingParameters params) { params_ = params; }

void SkidpadTrimming::fov_trimming(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
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

  if (params_.apply_rotation) {
    double theta = params_.rotation * M_PI / 180.0;

    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    for (size_t i = 0; i < num_points; ++i) {
      float* x_ptr = reinterpret_cast<float*>(&cloud_data[LidarPoint::PointX(i)]);
      float* y_ptr = reinterpret_cast<float*>(&cloud_data[LidarPoint::PointY(i)]);

      float x = *x_ptr;
      float y = *y_ptr;

      if (x == 0.0f && y == 0.0f) {
        continue;
      }

      const float rotated_x = static_cast<float>(x * cos_theta - y * sin_theta);
      const float rotated_y = static_cast<float>(x * sin_theta + y * cos_theta);

      *x_ptr = rotated_x;
      *y_ptr = rotated_y;
    }
  }

  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(i)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(i)]);

    if (x == 0.0 && y == 0.0 && z == 0.0) {
      continue;
    }

    if (within_limits(x, y, z, params_, params_.skid_max_range)) {
      uint8_t* out = &trimmed_cloud->data[trimmed_cloud->width * LidarPoint::POINT_STEP];
      std::memcpy(out, &cloud_data[LidarPoint::PointX(i)], LidarPoint::POINT_STEP);

      trimmed_cloud->width++;
    }
  }

  trimmed_cloud->data.resize(trimmed_cloud->width *
                             LidarPoint::POINT_STEP);  // Shrink to actual size
  trimmed_cloud->row_step = trimmed_cloud->width * LidarPoint::POINT_STEP;
}
