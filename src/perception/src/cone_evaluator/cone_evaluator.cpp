#include <cone_evaluator/cone_evaluator.hpp>

ConeEvaluator::ConeEvaluator(std::shared_ptr<EvaluatorParameters> params) : params_(params) {}

bool ConeEvaluator::close_to_ground(Cluster &cluster, const GroundGrid &ground_grid) const {
  Eigen::Vector4f centroid = cluster.get_centroid();

  auto &data = cluster.get_point_cloud()->data;
  auto &indices = cluster.get_point_indices();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();

  for (const auto &idx : indices) {
    float z = *reinterpret_cast<const float *>(&data[LidarPoint::PointZ(idx)]);
    min_z = std::min(z, min_z);
    max_z = std::max(z, max_z);
  }

  double ground_height = ground_grid.get_ground_height(centroid.x(), centroid.y());

  if (std::isnan(ground_height)) {
    return false;
  }

  double max_distance_above = max_z - ground_height;
  double min_distance_above = min_z - ground_height;

  bool result = true;

  result &= min_distance_above < params_->max_distance_from_ground_min;

  result &= max_distance_above < params_->max_distance_from_ground_max;

  return result;
}

bool ConeEvaluator::cylinder_fits_cone(Cluster &cluster) const {
  const auto &cloud_data = cluster.get_point_cloud()->data;
  const auto &indices = cluster.get_point_indices();
  const auto &centroid = cluster.get_centroid();

  int n_out_points = 0;
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();

  // Loop over points in the cluster
  for (size_t idx : indices) {
    float x = *reinterpret_cast<const float *>(&cloud_data[LidarPoint::PointX(idx)]);
    float y = *reinterpret_cast<const float *>(&cloud_data[LidarPoint::PointY(idx)]);
    float z = *reinterpret_cast<const float *>(&cloud_data[LidarPoint::PointZ(idx)]);

    min_z = std::min(static_cast<double>(z), min_z);
    max_z = std::max(static_cast<double>(z), max_z);

    double dx = x - centroid.x();
    double dy = y - centroid.y();
    double dz = std::abs(z - centroid.z());

    double distanceXY = std::sqrt(dx * dx + dy * dy);

    // Choose radius/height depending on cone size
    double radius = 0;
    double half_height = 0;
    if (cluster.get_is_large()) {
      radius = params_->large_cone_width / 2.0;
      half_height = params_->large_cone_height / 2.0;
    } else {
      radius = params_->small_cone_width / 2.0;
      half_height = params_->small_cone_height / 2.0;
    }

    if (distanceXY > radius || dz > half_height) {
      n_out_points++;
    }
  }

  // Safety check for flat clusters (probably erros from ground removal)
  double height = max_z - min_z;
  double distance = std::sqrt(centroid.x() * centroid.x() + centroid.y() * centroid.y());
  if (distance < 15.0 && height < 0.05) {
    return false;
  }

  // Compute ratio of points outside the expected cylinder
  double out_ratio = static_cast<double>(n_out_points) / indices.size();

  // Validation passes if most points are within expected cone shape
  return out_ratio < params_->n_out_points_ratio;
}

bool ConeEvaluator::npoints_valid(Cluster &cluster) const {
  Eigen::Vector4f center = cluster.get_centroid();
  double distance = std::sqrt(center.x() * center.x() + center.y() * center.y());
  double n_points = static_cast<double>(cluster.get_point_indices().size());

  // Max and min number of points based on distance
  double max_points = std::max(
      params_->n_points_intial_max - (distance * params_->n_points_max_distance_reduction), 4.0);
  double min_points = std::max(
      params_->n_points_intial_min - (distance * params_->n_points_min_distance_reduction), 1.0);

  return n_points <= max_points && n_points >= min_points;
}

bool ConeEvaluator::evaluateCluster(Cluster &cluster, const GroundGrid &ground_grid) {
  bool result = true;

  result &= close_to_ground(cluster, ground_grid);

  result &= cylinder_fits_cone(cluster);

  result &= npoints_valid(cluster);

  return result;
}