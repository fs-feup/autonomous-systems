#include "ground_removal/constrained_ransac.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

ConstrainedRANSAC::ConstrainedRANSAC(const double epsilon, const int n_tries,
                                     const double plane_angle_diff)
    : epsilon(epsilon), n_tries(n_tries), plane_angle_diff(plane_angle_diff) {}

void ConstrainedRANSAC::ground_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane,
                                       [[maybe_unused]] const SplitParameters split_params) const {
  if (point_cloud->points.size() < 3) {
    RCLCPP_INFO(rclcpp::get_logger("ConstrainedGridRANSAC"),
                "Point cloud has less than 3 points, skipping ground removal.");
    *ret = *point_cloud;
  } else {
    // Calculate the best plane, the plane parameter is used as a default plane if no better plane
    // is found
    Plane default_plane = plane;
    plane = calculate_plane(point_cloud, default_plane);

    // Clear the output point cloud
    ret->clear();
    ret->header = point_cloud->header;
    ret->width = 0;
    ret->height = 1;
    ret->is_dense = point_cloud->is_dense;

    // Remove ground points based on the best plane
    for (const auto& point : point_cloud->points) {
      double distance = distance_to_plane(point, plane);
      if (distance >= epsilon) {  // Keep non-ground points
        ret->points.push_back(point);
        ret->width++;
      }
    }
  }
}

Plane ConstrainedRANSAC::calculate_plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                                         const Plane& target_plane) const {
  std::random_device rd;
  std::mt19937 gen(rd());

  Plane best_plane;
  int best_plane_inliers = 0;
  double best_plane_max_deviation = 0.0;
  bool best_plane_found = false;
  int n_skips = 0;

  for (int i = 0; i < n_tries; ++i) {
    // Randomly sample 3 different points
    std::vector<pcl::PointXYZI> sampled_points(3);
    std::vector<int> indices(3);

    indices = pick_3_random_indices(static_cast<int>(point_cloud->points.size()), gen);
    sampled_points[0] = point_cloud->points[indices[0]];
    sampled_points[1] = point_cloud->points[indices[1]];
    sampled_points[2] = point_cloud->points[indices[2]];

    // Calculate plane from 3 points
    Plane candidate_plane = fit_plane_to_points(sampled_points);

    // Check angle constraint if target plane is provided
    if (target_plane.get_a() != 0 || target_plane.get_b() != 0 || target_plane.get_c() != 0) {
      double angle_diff = calculate_angle_difference(candidate_plane, target_plane);

      // Skip if angle difference is bigger than defined threshold
      if (angle_diff > plane_angle_diff) {
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

    } else if (inliers == best_plane_inliers && max_deviation < best_plane_max_deviation) {
      best_plane = candidate_plane;
      best_plane_inliers = inliers;
      best_plane_max_deviation = max_deviation;
    } else {
      // No update
    }
  }

  if (!best_plane_found || n_skips == n_tries) {
    best_plane = target_plane;
  }
  return best_plane;
}

Plane ConstrainedRANSAC::fit_plane_to_points(const std::vector<pcl::PointXYZI>& points) const {
  const auto& p1 = points[0];
  const auto& p2 = points[1];
  const auto& p3 = points[2];

  // Calculate two vectors from the points
  Eigen::Vector3d v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
  Eigen::Vector3d v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

  // Calculate normal vector using cross product
  Eigen::Vector3d normal = v1.cross(v2);
  double nrm = normal.norm();

  Plane plane(0.0, 0.0, 0.0, 0.0);
  // Check for case where points are collinear
  if (nrm >= 1e-12) {
    normal /= nrm;
    plane = Plane(normal.x(), normal.y(), normal.z(),
                  -(normal.x() * p1.x + normal.y() * p1.y + normal.z() * p1.z));
  }

  return plane;
}

double ConstrainedRANSAC::distance_to_plane(const pcl::PointXYZI& point, const Plane& plane) const {
  // Plane equation: Ax + By + Cz + D = 0
  double A = plane.get_a();
  double B = plane.get_b();
  double C = plane.get_c();
  double D = plane.get_d();

  double denom = std::sqrt(A * A + B * B + C * C);
  double result;
  // Avoid possible division by zero
  if (denom < 1e-12) {
    result = std::numeric_limits<double>::infinity();
  } else {
    result = std::abs(A * point.x + B * point.y + C * point.z + D) / denom;
  }
  return result;
}

double ConstrainedRANSAC::calculate_angle_difference(const Plane& plane1,
                                                     const Plane& plane2) const {
  Eigen::Vector3d normal1(plane1.get_a(), plane1.get_b(), plane1.get_c());
  Eigen::Vector3d normal2(plane2.get_a(), plane2.get_b(), plane2.get_c());
  normal1.normalize();
  normal2.normalize();

  // Calculate dot product
  double dot_product = normal1.dot(normal2);
  dot_product = std::clamp(dot_product, -1.0, 1.0);

  // Calculate angle between the normals
  double radian_angle = std::acos(std::abs(dot_product));
  return radian_angle * (180.0 / M_PI);  // Convert to degrees
}

std::vector<int> ConstrainedRANSAC::pick_3_random_indices(int max_index, std::mt19937& gen) const {
  std::uniform_int_distribution dis1(0, max_index - 1);
  int i1 = dis1(gen);

  // Do not choose the last element, because we may need to increment it
  std::uniform_int_distribution dis2(0, max_index - 2);
  int i2 = dis2(gen);
  if (i2 == i1) {
    ++i2;
  }

  // Do not choose the last two elements, because we may need to increment it twice
  std::uniform_int_distribution dis3(0, max_index - 3);
  int i3 = dis3(gen);
  if (i3 == i1 || i3 == i2) {
    ++i3;
  }
  if (i3 == i1 || i3 == i2) {
    ++i3;
  }

  return {i1, i2, i3};
}