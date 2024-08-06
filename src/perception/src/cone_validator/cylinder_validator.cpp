#include <cone_validator/cylinder_validator.hpp>

CylinderValidator::CylinderValidator(double width, double height) : width(width), height(height) {}

double CylinderValidator::getRadius() const { return std::sqrt(2 * width * width) / 2; }

bool CylinderValidator::coneValidator(Cluster* cone_point_cloud, [[maybe_unused]] Plane& plane) const {
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud = cone_point_cloud->get_point_cloud();

  for (const auto& point : *cone_point_cloud->get_point_cloud()) {
    // Calculate the distance between the point and the cylinder's centroid
    double distanceXY = std::sqrt((point.x - cone_point_cloud->get_centroid().x()) *
                                      (point.x - cone_point_cloud->get_centroid().x()) +
                                  (point.y - cone_point_cloud->get_centroid().y()) *
                                      (point.y - cone_point_cloud->get_centroid().y()));

    // Calculate distance between the point and the centroid along the z-axis
    double distanceZ = std::abs(point.z - cone_point_cloud->get_centroid().z());

    // If the point is outside the cylinder, return false
    if (distanceXY > getRadius() || distanceZ > height / 2) return false;
  }

  // All points are inside the cylinder
  return true;
}