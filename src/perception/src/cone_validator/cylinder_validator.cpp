#include <cone_validator/cylinder_validator.hpp>

CylinderValidator::CylinderValidator(double small_width, double small_height, double large_width,
                                     double large_height, double out_distance_cap)
    : small_width(small_width),
      small_height(small_height),
      large_width(large_width),
      large_height(large_height),
      out_distance_cap(out_distance_cap) {}

double CylinderValidator::small_getRadius() const {
  return std::sqrt(2 * small_width * small_width) / 2;
}

double CylinderValidator::large_getRadius() const {
  return std::sqrt(2 * large_width * large_width) / 2;
}

void CylinderValidator::coneValidator(Cluster *cone_point_cloud, EvaluatorResults *results,
                                      [[maybe_unused]] Plane &plane) const {
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud = cone_point_cloud->get_point_cloud();

  double out_distanceXY_small = 1, out_distanceXY_large = 1;
  double out_distanceZ_small = 1, out_distanceZ_large = 1;
  double n_large_points = 0;
  double n_out_points = 0;

  pcl::PointXYZ center(cone_point_cloud->get_centroid().x(), cone_point_cloud->get_centroid().y(),
                       cone_point_cloud->get_centroid().z());
  for (const auto &point : *point_cloud) {
    // Calculate the distance between the point and the cylinder's centroid
    double distanceXY = std::sqrt((point.x - center.x) * (point.x - center.x) +
                                  (point.y - center.y) * (point.y - center.y));

    // Calculate distance between the point and the centroid along the z-axis
    double distanceZ = std::abs(point.z - center.z);

    if (distanceXY > small_getRadius() || distanceZ > small_height / 2) {
      out_distanceXY_small = std::min(out_distanceXY_small, small_getRadius() / distanceXY);
      out_distanceZ_small = std::min(out_distanceZ_small, small_height / (2 * distanceZ));

      if (distanceXY <= large_getRadius() && distanceZ <= large_height / 2) {
        n_large_points++;
      } else {
        n_out_points++;
        out_distanceXY_large = std::min(out_distanceXY_large, large_getRadius() / distanceXY);
        out_distanceZ_large = std::min(out_distanceZ_large, large_height / (2 * distanceZ));
      }
    }
  }
  out_distanceXY_large = out_distanceXY_large >= out_distance_cap ? out_distanceXY_large : 0.0;
  out_distanceZ_large = out_distanceZ_large >= out_distance_cap ? out_distanceZ_large : 0.0;

  out_distanceXY_small = out_distanceXY_small >= out_distance_cap ? out_distanceXY_small : 0.0;
  out_distanceZ_small = out_distanceZ_small >= out_distance_cap ? out_distanceZ_small : 0.0;

  results->cylinder_out_distance_xy_large = out_distanceXY_large;
  results->cylinder_out_distance_z_large = out_distanceZ_large;
  results->cylinder_out_distance_xy_small = out_distanceXY_small;
  results->cylinder_out_distance_z_small = out_distanceZ_small;
  results->cylinder_n_large_points = n_large_points;
  results->cylinder_n_out_points = n_out_points;
}