#include "ground_removal/ransac2.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>

RANSAC2::RANSAC2(const double epsilon, const int n_tries, const double plane_angle_diff)
    : epsilon(epsilon), n_tries(n_tries), plane_angle_diff(plane_angle_diff) {}

void RANSAC2::ground_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane,
                             [[maybe_unused]] const SplitParameters split_params) const {
  auto start_time = std::chrono::high_resolution_clock::now();

  if (point_cloud->points.size() < 3) {
    throw std::invalid_argument("Point cloud must contain at least 3 points to fit a plane.");
  }
  Plane default_plane = plane;

  // Calculate the best plane
  Plane best_plane = calculate_plane(point_cloud, default_plane);

  plane = best_plane;

  // Clear the output point cloud
  ret->clear();
  ret->header = point_cloud->header;
  ret->width = 0;
  ret->height = 1;
  ret->is_dense = point_cloud->is_dense;

  // Remove ground points based on the best plane
  for (const auto& point : point_cloud->points) {
    double distance = distance_to_plane(point, best_plane);
    if (distance >= epsilon) {  // Keep non-ground points
      ret->points.push_back(point);
      ret->width++;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
}

Plane RANSAC2::calculate_plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                               const Plane& target_plane) const {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, point_cloud->points.size() - 1);

  Plane best_plane;
  int best_plane_inliers = 0;
  double best_plane_max_deviation = 0.0;
  bool best_plane_found = false;
  double best_angle_diff;
  int n_skips = 0;

  for (int i = 0; i < n_tries; ++i) {
    // Randomly sample 3 different points
    std::vector<pcl::PointXYZI> sampled_points(3);
    std::vector<int> indices(3);

    // Ensure 3 different points
    do {
      for (int j = 0; j < 3; ++j) {
        indices[j] = dis(gen);
        sampled_points[j] = point_cloud->points[indices[j]];
      }
    } while (indices[0] == indices[1] || indices[0] == indices[2] || indices[1] == indices[2]);

    // Calculate plane from 3 points
    Plane candidate_plane = fit_plane_to_points(sampled_points);

    // Check angle constraint if target plane is provided
    double angle_diff_degrees;
    if (target_plane.get_a() != 0 || target_plane.get_b() != 0 || target_plane.get_c() != 0) {
      double angle_diff = calculate_angle_difference(candidate_plane, target_plane);
      angle_diff_degrees = angle_diff * (180.0 / M_PI);

      // Skip if angle difference is bigger than defined threshold
      if (angle_diff_degrees > plane_angle_diff || std::abs(candidate_plane.get_d()) > 1.80 ||
          std::abs(candidate_plane.get_d()) < 0.80) {
        n_skips++;
        continue;
      }
    }

    // Count inliers and calculate max deviation
    int inliers = 0;
    double max_deviation = 0.0;

    for (const auto& point : point_cloud->points) {
      double distance = distance_to_plane(point, candidate_plane);
      if (distance < epsilon) {
        inliers++;
        if (distance > max_deviation) {
          max_deviation = distance;
        }
      }
    }

    // Update best plane based on inliers count and max deviation
    if (!best_plane_found || inliers > best_plane_inliers) {
      best_plane = candidate_plane;
      best_plane_inliers = inliers;
      best_plane_max_deviation = max_deviation;
      best_plane_found = true;
      best_angle_diff = angle_diff_degrees;

    } else if (inliers == best_plane_inliers && max_deviation < best_plane_max_deviation) {
      best_plane = candidate_plane;
      best_plane_inliers = inliers;
      best_plane_max_deviation = max_deviation;
      best_angle_diff = angle_diff_degrees;
    }
  }
  if (!best_plane_found || n_skips == n_tries) {
    return target_plane;
  }

  return best_plane;
}

Plane RANSAC2::fit_plane_to_points(const std::vector<pcl::PointXYZI>& points) const {
  const auto& p1 = points[0];
  const auto& p2 = points[1];
  const auto& p3 = points[2];

  // Calculate two vectors from the points
  Eigen::Vector3d v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
  Eigen::Vector3d v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

  // Calculate normal vector using cross product
  Eigen::Vector3d normal = v1.cross(v2);
  normal.normalize();

  Plane plane(normal.x(), normal.y(), normal.z(),
              -(normal.x() * p1.x + normal.y() * p1.y + normal.z() * p1.z));

  return plane;
}

double RANSAC2::distance_to_plane(const pcl::PointXYZI& point, const Plane& plane) const {
  // Plane equation: Ax + By + Cz + D = 0
  double A = plane.get_a();
  double B = plane.get_b();
  double C = plane.get_c();
  double D = plane.get_d();

  return std::abs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);
}

double RANSAC2::calculate_angle_difference(const Plane& plane1, const Plane& plane2) const {
  Eigen::Vector3d normal1(plane1.get_a(), plane1.get_b(), plane1.get_c());
  Eigen::Vector3d normal2(plane2.get_a(), plane2.get_b(), plane2.get_c());
  normal1.normalize();
  normal2.normalize();

  // Calculate dot product
  double dot_product = normal1.dot(normal2);
  dot_product = std::clamp(dot_product, -1.0, 1.0);

  // Calculate angle between the normals
  return std::acos(std::abs(dot_product));
}