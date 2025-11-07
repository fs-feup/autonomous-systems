#include <cone_evaluator/cone_evaluator.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>

ConeEvaluator::ConeEvaluator(std::shared_ptr<EvaluatorParameters> params) : params_(params) {}

bool ConeEvaluator::close_to_ground(Cluster &cluster, const GroundGrid &ground_grid) const {
  Eigen::Vector4f centroid = cluster.get_centroid();

  auto &data = cluster.get_point_cloud()->data;
  auto &indices = cluster.get_point_indices();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();

  for (int i = 0; i < indices.size(); i++) {
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
  bool result = false;

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
    double height =
        cluster.get_is_large() ? params_->large_cone_height : params_->small_cone_height;

    if (distanceXY > radius || dz > height / 2.0) n_out_points++;
  }

  // Compute ratio of points outside the expected cylinder
  double out_ratio = static_cast<double>(n_out_points) / indices.size();

  // Validation passes if most points are within expected cone shape
  return out_ratio < params_->n_out_points_ratio;
}

bool ConeEvaluator::npoints_valid(Cluster &cluster) const {
  Eigen::Vector4f center = cluster.get_centroid();

  // Extract ground-plane distance
  double distance = std::sqrt(center.x() * center.x() + center.y() * center.y());

  // === Parameters ===
  const double lidar_height = params_->lidar_height;              // [m]
  const double v_res_deg = params_->lidar_vertical_resolution;    // [deg]
  const double h_res_deg = params_->lidar_horizontal_resolution;  // [deg]
  const double cone_height = params_->large_cone_height;          // [m]
  const double cone_width = params_->large_cone_width;            // [m]
  const double visibility = params_->visibility_factor;           // (0–1)
  const double threshold = params_->expected_points_threshold;    // allowed deviation

  // === Compute vertical angular span (includes lidar height) ===
  double alpha_v =
      std::atan2(cone_height - lidar_height, distance) - std::atan2(-lidar_height, distance);

  // === Compute horizontal angular span ===
  double beta_h = 2.0 * std::atan2(cone_width / 2.0, distance);

  // === Convert resolutions to radians ===
  double v_res = v_res_deg * M_PI / 180.0;
  double h_res = h_res_deg * M_PI / 180.0;

  // === Estimate number of beams hitting the cone ===
  double n_v = alpha_v / v_res;
  double n_h = beta_h / h_res;

  // If cone outside vertical FOV, alpha_v could be negative
  if (n_v <= 0.0 || n_h <= 0.0) return false;

  // Expected number of returns (visibility factor)
  double expected_points = std::min(visibility * n_v * n_h, 35.0);

  // Actual number of points in the cluster
  double n_points = static_cast<double>(cluster.get_point_indices().size());

  // RCLCPP_INFO(rclcpp::get_logger("ConeEvaluator"),
  //             "Position: (%.2f, %.2f, %.2f), Expected points: %.2f, Actual points: %.2f",
  //             center.x(), center.y(), center.z(), expected_points, n_points);

  expected_points = 50 - (distance * 1.35);  // Adjust expected points based on distance

  return n_points <= (expected_points) && n_points >= (2);
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