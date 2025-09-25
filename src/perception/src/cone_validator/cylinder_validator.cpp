#include <cone_validator/cylinder_validator.hpp>

CylinderValidator::CylinderValidator(double small_width, double small_height, double large_width,
                                     double large_height, double out_distance_cap)
    : small_width(small_width),
      small_height(small_height),
      large_width(large_width),
      large_height(large_height),
      out_distance_cap(out_distance_cap) {}

double CylinderValidator::small_getRadius() const { return small_width / 2; }

double CylinderValidator::large_getRadius() const { return large_width / 2; }

std::vector<double> CylinderValidator::coneValidator(Cluster* cone_point_cloud,
                                                     [[maybe_unused]] Plane& plane) const {
  pcl::PointCloud<PointXYZIR>::Ptr point_cloud = cone_point_cloud->get_point_cloud();

  double out_distanceXY = 1;
  double out_distanceZ = 1;
  int n_out_points = 0;

  for (const auto& point : *cone_point_cloud->get_point_cloud()) {
    // Calculate the distance between the point and the cylinder's centroid
    double distanceXY = std::sqrt((point.x - cone_point_cloud->get_centroid().x()) *
                                      (point.x - cone_point_cloud->get_centroid().x()) +
                                  (point.y - cone_point_cloud->get_centroid().y()) *
                                      (point.y - cone_point_cloud->get_centroid().y()));

    // Calculate distance between the point and the centroid along the z-axis
    double distanceZ = std::abs(point.z - cone_point_cloud->get_centroid().z());

    // Choose which cylinder to use depending on the size of the cluster (decided in
    // height_validator).
    if (cone_point_cloud->get_is_large() &&
        (distanceXY > large_getRadius() || distanceZ > large_height / 2)) {
      n_out_points++;
      out_distanceXY = std::min(out_distanceXY, large_getRadius() / distanceXY);
      out_distanceZ = std::min(out_distanceZ, large_height / (2 * distanceZ));

    } else if (distanceXY > small_getRadius() || distanceZ > small_height / 2) {
      n_out_points++;
      out_distanceXY = std::min(out_distanceXY, small_getRadius() / distanceXY);
      out_distanceZ = std::min(out_distanceZ, small_height / (2 * distanceZ));
    }
    out_distanceXY = out_distanceXY >= out_distance_cap ? out_distanceXY : 0.0;
    out_distanceZ = out_distanceZ >= out_distance_cap ? out_distanceZ : 0.0;
  }
  // index 0 = ratio of between distance to the farthest point and the cylinder radius.
  // index 1 = ratio of between distance to the farthest point and the cylinder heigth.
  // index 2 = ratio between the number of points outside the cylinder and the number of total
  // points.
  return {out_distanceXY, out_distanceZ, 1.0 - (double)n_out_points / point_cloud->size()};
}