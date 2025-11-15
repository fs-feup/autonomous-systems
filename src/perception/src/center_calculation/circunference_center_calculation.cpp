#include <center_calculation/circunference_center_calculation.hpp>

Eigen::Vector4f CircunferenceCenterCalculator::calculate_center(
    const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
    const std::vector<int>& point_indices, const Plane& plane) const {
  Eigen::Vector4f centroid;

  // Get the centroid cone of the point_cloud;
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

  // Get the parallel plane that passes through that point
  double d = -(plane.get_a() * centroid.x() + plane.get_b() * centroid.y() +
               plane.get_c() * centroid.z());  // d component of the plane

  Plane middle_plane(plane.get_a(), plane.get_b(), plane.get_c(), d);

  // Gets the 3 closest points to that plane
  std::vector<Eigen::Vector3f> closest_points(3, Eigen::Vector3f::Zero());
  std::vector<double> distances(3, std::numeric_limits<double>::max());

  for (int idx : point_indices) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(idx)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(idx)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(idx)]);

    Eigen::Vector3f point(x, y, z);
    double distance = middle_plane.get_distance_to_point(x, y, z);

    // Update distances/closest point -> Do the vector shifting and place the right points on the
    // vector
    for (int i = 0; i < 3; ++i) {
      if (distance < distances[i]) {
        for (int j = 2; j > i; --j) {
          closest_points[j] = closest_points[j - 1];
          distances[j] = distances[j - 1];
        }
        closest_points[i] = point;
        distances[i] = distance;
        break;
      }
    }
  }

  // Calculates the center of the circumference that passes on those 3 points
  // Source: https://codeforces.com/blog/entry/17313
  Eigen::Vector4f center;

  double x1 = closest_points[0].x();
  double y1 = closest_points[0].y();
  double x2 = closest_points[1].x();
  double y2 = closest_points[1].y();
  double x3 = closest_points[2].x();
  double y3 = closest_points[2].y();

  double D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
  center.x() = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                (x3 * x3 + y3 * y3) * (y1 - y2)) /
               D;
  center.y() = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                (x3 * x3 + y3 * y3) * (x2 - x1)) /
               D;
  center.z() = centroid.z();

  return center;
}
