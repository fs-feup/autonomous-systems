#include <cone_validator/displacement_validator.hpp>

DisplacementValidator::DisplacementValidator(double min_distance_x, double min_distance_y,
                                             double min_distance_z)
    : _min_distance_x_(min_distance_x),
      _min_distance_y_(min_distance_y),
      _min_distance_z_(min_distance_z) {}

std::vector<double> DisplacementValidator::coneValidator(Cluster* cone_cluster,
                                                         [[maybe_unused]] Plane& plane) const {
  const auto& cloud_data = cone_cluster->get_point_cloud()->data;
  const auto& indices = cone_cluster->get_point_indices();

  // Initialize min and max with the first point
  float minX = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(indices[0])]);
  float maxX = minX;
  float minY = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(indices[0])]);
  float maxY = minY;
  float minZ = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(indices[0])]);
  float maxZ = minZ;

  // Loop through all points to find min/max on each axis
  for (size_t i = 1; i < indices.size(); ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(indices[i])]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(indices[i])]);
    float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(indices[i])]);

    minX = std::min(minX, x);
    maxX = std::max(maxX, x);

    minY = std::min(minY, y);
    maxY = std::max(maxY, y);

    minZ = std::min(minZ, z);
    maxZ = std::max(maxZ, z);
  }

  // index 0 = ratio between the x axis displacement and the minimum distance for that axis.
  // index 1 = ratio between the y axis displacement and the minimum distance for that axis.
  // index 2 = ratio between the z axis displacement and the minimum distance for that axis.
  return {std::min((maxX - minX) / _min_distance_x_, 1.0),
          std::min((maxY - minY) / _min_distance_y_, 1.0),
          std::min((maxZ - minZ) / _min_distance_z_, 1.0)};
}