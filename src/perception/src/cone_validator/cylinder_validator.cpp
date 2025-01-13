#include <cone_validator/cylinder_validator.hpp>

CylinderValidator::CylinderValidator(double small_width, double small_height, double large_width,
                                     double large_height)
    : small_width(small_width),
      small_height(small_height),
      large_width(large_width),
      large_height(large_height) {}

double CylinderValidator::small_getRadius() const {
  return std::sqrt(2 * small_width * small_width) / 2;
}

double CylinderValidator::large_getRadius() const {
  return std::sqrt(2 * large_width * large_width) / 2;
}

bool CylinderValidator::coneValidator(Cluster* cone_point_cloud,
                                      [[maybe_unused]] Plane& plane) const {
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud = cone_point_cloud->get_point_cloud();

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
    if (cone_point_cloud->get_is_large()) {
      // If the point is outside the large cylinder, return false
      if (distanceXY > large_getRadius() || distanceZ > large_height / 2) return false;
    } else {
      // If the point is outside the small cylinder, return false
      if (distanceXY > small_getRadius() || distanceZ > small_height / 2) return false;
    }
  }

  // All points are inside the cylinder
  return true;
}