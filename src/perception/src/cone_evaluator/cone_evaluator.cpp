#include <cone_evaluator/cone_evaluator.hpp>

ConeEvaluator::ConeEvaluator(std::shared_ptr<EvaluatorParameters> params) : params_(params) {}

bool ConeEvaluator::close_to_ground(Cluster &cluster, const GroundGrid &ground_grid) const {
  Eigen::Vector4f centroid = cluster.get_centroid();

  auto &data = cluster.get_point_cloud()->data;
  auto &indices = cluster.get_point_indices();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();

  for (size_t i = 0; i < indices.size(); i++) {
    float z = *reinterpret_cast<const float *>(&data[PointZ(indices[i])]);
    min_z = std::min(z, min_z);
    max_z = std::max(z, max_z);
  }

  double ground_height = ground_grid.get_ground_height(centroid.x(), centroid.y());

  if (std::isnan(ground_height)) {
    return false;  // No ground data available for this grid cell
  }

  double max_distance_above = max_z - ground_height;
  double min_distance_above = min_z - ground_height;

  if (min_distance_above > params_->max_distance_from_ground_min) {
    return false;
  }
  if (max_distance_above > params_->max_distance_from_ground_max) {
    return false;
  }

  return true;
}

bool ConeEvaluator::cylinder_fits_cone(Cluster &cluster) const {
  const auto &cloud_data = cluster.get_point_cloud()->data;
  const auto &indices = cluster.get_point_indices();
  const auto &centroid = cluster.get_centroid();

  int n_out_points = 0;

  // Loop over points in the cluster
  for (size_t idx : indices) {
    float x = *reinterpret_cast<const float *>(&cloud_data[PointX(idx)]);
    float y = *reinterpret_cast<const float *>(&cloud_data[PointY(idx)]);
    float z = *reinterpret_cast<const float *>(&cloud_data[PointZ(idx)]);

    double dx = x - centroid.x();
    double dy = y - centroid.y();
    double dz = std::abs(z - centroid.z());

    double distanceXY = std::sqrt(dx * dx + dy * dy);

    // Choose radius/height depending on cone size
    double radius =
        cluster.get_is_large() ? params_->large_cone_width / 2.0 : params_->small_cone_width / 2.0;
    double half_height = cluster.get_is_large() ? params_->large_cone_height / 2.0
                                                : params_->small_cone_height / 2.0;

    if (distanceXY > radius || dz > half_height) {
      n_out_points++;
    }
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

  // Approximation of expected number of points based on distance
  double expected_points = std::max(50 - (distance * 1.35), 1.0);
  double min_points = 2;  // Needs to be changed

  return n_points <= (expected_points) && n_points >= (min_points);
}

bool ConeEvaluator::evaluateCluster(Cluster &cluster, const GroundGrid &ground_grid) {
  if (!close_to_ground(cluster, ground_grid)) {
    return false;
  }
  if (!cylinder_fits_cone(cluster)) {
    return false;
  }
  if (!npoints_valid(cluster)) {
    return false;
  }
  return true;
}