#include <cone_evaluator/cone_evaluator.hpp>

ConeEvaluator::ConeEvaluator(std::shared_ptr<EvaluatorParameters> params) : params_(params) {}

bool ConeEvaluator::evaluateCluster(Cluster &cluster, Plane &ground_plane) {
  double confidence = 0;
  EvaluatorResults results;

  // Height
  params_->height_validator->coneValidator(&cluster, &results, ground_plane);
  // Cylinder
  params_->cylinder_validator->coneValidator(&cluster, &results, ground_plane);
  // NPoints
  params_->npoints_validator->coneValidator(&cluster, &results, ground_plane);
  // Displacement
  params_->displacement_validator->coneValidator(&cluster, &results, ground_plane);
  // Deviation
  params_->deviation_validator->coneValidator(&cluster, &results, ground_plane);

  // Define if the cone is large or small
  if (results.cylinder_n_large_points >= params_->min_large_points && results.height_large) {
    cluster.set_is_large();
  }

  // NPoints
  confidence += params_->npoints_weight * results.n_points;

  // Deviation
  confidence += params_->deviation_xoy_weight * results.deviation_xoy +
                params_->deviation_z_weight * results.deviation_z;

  // Displacement
  confidence += params_->displacement_x_weight * results.displacement_x +
                params_->displacement_y_weight * results.displacement_y +
                params_->displacement_z_weight * results.displacement_z;

  if (cluster.get_is_large()) {
    // Height
    confidence += params_->height_out_weight * results.height_out_ratio_large +
                  params_->height_in_weight * results.height_in_ratio_large;

    // Cylinder
    confidence += params_->cylinder_height_weight * results.cylinder_out_distance_z_large +
                  params_->cylinder_radius_weight * results.cylinder_out_distance_xy_large +
                  params_->cylinder_npoints_weight *
                      (1 - (results.cylinder_n_out_points / cluster.get_point_cloud()->size()));
  } else {
    // Height
    confidence += params_->height_out_weight * results.height_out_ratio_small +
                  params_->height_in_weight * results.height_in_ratio_small;

    // Cylinder
    confidence += params_->cylinder_height_weight * results.cylinder_out_distance_z_small +
                  params_->cylinder_radius_weight * results.cylinder_out_distance_xy_small +
                  params_->cylinder_npoints_weight *
                      (1 - ((results.cylinder_n_out_points + results.cylinder_n_large_points) /
                            cluster.get_point_cloud()->size()));
  }

  // Adjust for precision issues [0,1]
  confidence = std::max(0.0, std::min(confidence, 1.0));

  cluster.set_confidence(confidence);
  return confidence >= params_->min_confidence;
}