#include <cone_validator/height_validator.hpp>

HeightValidator::HeightValidator(double min_height, double large_max_height,
                                 double small_max_height, double height_cap)
    : _min_height_(min_height),
      _large_max_height_(large_max_height),
      _small_max_height_(small_max_height),
      _height_cap_(height_cap) {}

std::vector<double> HeightValidator::coneValidator(Cluster* cone_cluster, Plane& plane) const {
  const auto& cloud_data = cone_cluster->get_point_cloud()->data;
  const auto& indices = cone_cluster->get_point_indices();

  // Initialize maxZ with the distance of the first point to the plane
  float x0 = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(indices[0])]);
  float y0 = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(indices[0])]);
  float z0 = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(indices[0])]);

  double maxZ = plane.get_distance_to_point(x0, y0, z0);

  // Track the point corresponding to maxZ (optional)
  Eigen::Vector3f maxPoint(x0, y0, z0);

  for (size_t i = 1; i < indices.size(); ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(indices[i])]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(indices[i])]);
    float z = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointZ(indices[i])]);

    double distance = plane.get_distance_to_point(x, y, z);
    if (distance > maxZ) {
      maxZ = distance;
      maxPoint = Eigen::Vector3f(x, y, z);
    }
  }

  std::vector<double> res;
  double r1 = 0, r2 = 0;
  if (maxZ > _small_max_height_) {
    cone_cluster->set_is_large();

    if (_min_height_ > 0) {
      r1 = std::min({_large_max_height_ / maxZ, maxZ / _min_height_, 1.0});
    } else {
      r1 = std::min({_large_max_height_ / maxZ, 1.0});
    }

    if (r1 == 1.0) {
      r2 = maxZ / _large_max_height_;
    } else {
      r2 = 0.0;
    }
    res.push_back(r1);
    res.push_back(r2);
  } else {
    if (_min_height_ > 0) {
      r1 = std::min({_small_max_height_ / maxZ, maxZ / _min_height_, 1.0});
    } else {
      r1 = std::min({_small_max_height_ / maxZ, 1.0});
    }

    if (r1 == 1.0) {
      r2 = maxZ / _small_max_height_;
    } else {
      r2 = 0.0;
    }
    res.push_back(r1);
    res.push_back(r2);
  }

  if (res[0] < _height_cap_) {
    res[0] = 0.0;
  }

  // index 0 = if not in height interval, ratio between the height of the cluster and the maximum or
  // minimum height, whichever is closest to.
  // index 1 = if in height interval, how close is it to the maximum height, else 0.
  return res;
}
