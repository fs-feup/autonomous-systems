#include "planning/smoothing.hpp"

void PathSmoothing::smooth_path(std::vector<PathPoint>& unordered_path, std::vector<PathPoint>& yellow_cones, std::vector<PathPoint>& blue_cones) const {
  if (config_.use_path_smoothing_) {
    // auto spline = fit_spline(config_.precision_, config_.order_, config_.coeffs_ratio_, unordered_path);
    // unordered_path = spline;
    auto splines = fitTripleSpline(unordered_path, blue_cones, yellow_cones, config_.precision_, config_.order_, config_.coeffs_ratio_);
    unordered_path = splines.center;
    yellow_cones = splines.right;
    blue_cones = splines.left;
  } 
}