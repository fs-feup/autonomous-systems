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

std::vector<double> CylinderValidator::coneValidator(Cluster* cone_cluster,
                                                     [[maybe_unused]] Plane& plane) const {
  const auto& cloud_data = cone_cluster->get_point_cloud()->data;
  const auto& indices = cone_cluster->get_point_indices();

  double out_distanceXY = 1.0;
  double out_distanceZ = 1.0;
  int n_out_points = 0;

  // Loop over points in the cluster
  for (size_t idx : indices) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(idx)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(idx)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(idx)]);

    // Calculate the distance between the point and the cylinder's centroid
    double distanceXY =
        std::sqrt((x - cone_cluster->get_centroid().x()) * (x - cone_cluster->get_centroid().x()) +
                  (y - cone_cluster->get_centroid().y()) * (y - cone_cluster->get_centroid().y()));

    // Calculate distance between the point and the centroid along the z-axis
    double distanceZ = std::abs(z - cone_cluster->get_centroid().z());

    // Choose which cylinder to use depending on the size of the cluster
    if (cone_cluster->get_is_large() &&
        (distanceXY > large_getRadius() || distanceZ > large_height / 2)) {
      n_out_points++;
      out_distanceXY = std::min(out_distanceXY, large_getRadius() / distanceXY);
      out_distanceZ = std::min(out_distanceZ, large_height / (2 * distanceZ));

    } else if (distanceXY > small_getRadius() || distanceZ > small_height / 2) {
      n_out_points++;
      out_distanceXY = std::min(out_distanceXY, small_getRadius() / distanceXY);
      out_distanceZ = std::min(out_distanceZ, small_height / (2 * distanceZ));
    } else {
      // Point is inside the cylinder
      out_distanceXY = 1.0;
      out_distanceZ = 1.0;
    }

    // Apply cap thresholds
    if (out_distanceXY < out_distance_cap) {
      out_distanceXY = 0.0;
    }
    if (out_distanceZ < out_distance_cap) {
      out_distanceZ = 0.0;
    }
  }

  // index 0 = ratio of distance to the farthest point / cylinder radius
  // index 1 = ratio of distance to the farthest point / cylinder height
  // index 2 = ratio of points outside the cylinder
  return {out_distanceXY, out_distanceZ, 1.0 - static_cast<double>(n_out_points) / indices.size()};
}
