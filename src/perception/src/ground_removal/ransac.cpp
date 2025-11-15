#include "ground_removal/ransac.hpp"

#include <Eigen/Dense>
#include <random>

RANSAC::RANSAC(const double epsilon, const int n_tries) : epsilon(epsilon), n_tries(n_tries) {}

void RANSAC::ground_removal(const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
                            sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_cloud,
                            GroundGrid& ground_grid) const {
  const auto& cloud_data = trimmed_point_cloud->data;
  const size_t num_points = trimmed_point_cloud->width * trimmed_point_cloud->height;
  if (num_points < 3) {
    return;
  }

  ground_removed_cloud->header = trimmed_point_cloud->header;
  ground_removed_cloud->height = 1;
  ground_removed_cloud->is_bigendian = trimmed_point_cloud->is_bigendian;
  ground_removed_cloud->is_dense = true;
  ground_removed_cloud->fields = trimmed_point_cloud->fields;
  ground_removed_cloud->point_step = trimmed_point_cloud->point_step;
  ground_removed_cloud->width = 0;
  ground_removed_cloud->data.resize(num_points * trimmed_point_cloud->point_step);

  std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<size_t> dist(0, num_points - 1);

  double best_a = 0, best_b = 0, best_c = 0, best_d = 0;
  size_t best_inliers = 0;

  for (int i = 0; i < n_tries; ++i) {
    // Select 3 random points
    size_t idx1 = dist(rng), idx2 = dist(rng), idx3 = dist(rng);
    if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
      i--;
      continue;
    }

    Eigen::Vector3f p1 = *reinterpret_cast<const Eigen::Vector3f*>(&cloud_data[LidarPoint::PointX(idx1)]);
    Eigen::Vector3f p2 = *reinterpret_cast<const Eigen::Vector3f*>(&cloud_data[LidarPoint::PointX(idx2)]);
    Eigen::Vector3f p3 = *reinterpret_cast<const Eigen::Vector3f*>(&cloud_data[LidarPoint::PointX(idx3)]);

    Eigen::Vector3f v1 = p2 - p1;
    Eigen::Vector3f v2 = p3 - p1;
    Eigen::Vector3f n = v1.cross(v2);

    if (n.norm() < 1e-6) {
      continue;
    }
    n.normalize();
    double a = n.x(), b = n.y(), c = n.z();
    double d = -n.dot(p1);

    size_t inliers = 0;
    for (size_t j = 0; j < num_points; ++j) {
      float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(j)]);
      float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(j)]);
      float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(j)]);
      double dist = std::abs(a * x + b * y + c * z + d);
      if (dist < epsilon) {
        inliers++;
      }
    }

    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_a = a;
      best_b = b;
      best_c = c;
      best_d = d;
    }
  }

  // Classify points based on the best plane
  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(i)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(i)]);
    double dist = std::abs(best_a * x + best_b * y + best_c * z + best_d);
    if (dist < epsilon) {
      // Ground point, update ground grid
      ground_grid.set_ground_height(x, y, z);
    } else {
      // Non-ground
      size_t write_idx = ground_removed_cloud->width++;
      std::memcpy(&ground_removed_cloud->data[write_idx * trimmed_point_cloud->point_step],
                  &cloud_data[i * trimmed_point_cloud->point_step],
                  trimmed_point_cloud->point_step);
    }
  }

  ground_removed_cloud->data.resize(ground_removed_cloud->width * trimmed_point_cloud->point_step);
  ground_removed_cloud->row_step = ground_removed_cloud->width * trimmed_point_cloud->point_step;
}