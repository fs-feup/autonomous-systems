#include <center_calculation/centroid_calculation.hpp>

Eigen::Vector4f CentroidCalculator::calculate_center(
    const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
    const std::vector<int>& point_indices, const Plane& plane) const {
  Eigen::Vector4f centroid;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;

  const auto& cloud_data = point_cloud->data;

  for (int idx : point_indices) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(idx)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(idx)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(idx)]);

    sum_x += x;
    sum_y += y;
    sum_z += z;
  }

  size_t n = point_indices.size();
  centroid << sum_x / n, sum_y / n, sum_z / n, 1.0f;
  return centroid;
}