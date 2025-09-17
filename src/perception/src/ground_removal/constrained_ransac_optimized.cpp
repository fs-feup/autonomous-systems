#include "ground_removal/constrained_ransac_optimized.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "ground_removal/ransac.hpp"

ConstrainedRANSACOptimized::ConstrainedRANSACOptimized(const double epsilon, const int n_tries,
                                                       const double plane_angle_diff)
    : epsilon(epsilon), n_tries(n_tries), plane_angle_diff(plane_angle_diff) {}

void ConstrainedRANSACOptimized::ground_removal(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane,
    [[maybe_unused]] const SplitParameters split_params) const {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_indices(new pcl::PointIndices);
  Plane default_plane = plane;

  // Segmentation Object creation
  pcl::SACSegmentation<pcl::PointXYZI> seg;

  // Optional: Increases Accuracy
  seg.setOptimizeCoefficients(true);

  // Defining RANSAC properties in the segmentation Object
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(epsilon);
  seg.setMaxIterations(n_tries);

  seg.setInputCloud(point_cloud);
  seg.segment(*inliers_indices, *coefficients);

  if (coefficients->values.size() >= 4) {
    plane = Plane(coefficients->values[0], coefficients->values[1], coefficients->values[2],
                  coefficients->values[3]);

    // If no default plane provided, or angle check is okay, use candidate plane, if not, use
    // default
    if ((default_plane.get_a() == 0.0 && default_plane.get_b() == 0.0 &&
         default_plane.get_c() == 0.0 && default_plane.get_d() == 0.0) ||
        (calculate_angle_difference(plane, default_plane) <= plane_angle_diff)) {
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(point_cloud);
      extract.setIndices(inliers_indices);

      extract.setNegative(true);  // Extract outliers
      extract.filter(*ret);
      return;
    }
  }

  plane = default_plane;

  ret->clear();
  ret->header = point_cloud->header;
  ret->width = 0;
  ret->height = 1;
  ret->is_dense = point_cloud->is_dense;

  for (const auto& point : *point_cloud) {
    double distance = distance_to_plane(point, default_plane);
    if (distance > epsilon) {
      ret->points.push_back(point);
    }
  }
}

double ConstrainedRANSACOptimized::calculate_angle_difference(const Plane& plane1,
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

double ConstrainedRANSACOptimized::distance_to_plane(const pcl::PointXYZI& point,
                                                     const Plane& plane) const {
  // Plane equation: Ax + By + Cz + D = 0
  double A = plane.get_a();
  double B = plane.get_b();
  double C = plane.get_c();
  double D = plane.get_d();

  return std::abs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);
}