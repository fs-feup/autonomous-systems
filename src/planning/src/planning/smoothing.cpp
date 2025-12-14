#include "planning/smoothing.hpp"

std::vector<PathPoint> PathSmoothing::smooth_path(std::vector<PathPoint>& unordered_path) const {
  if (config_.use_path_smoothing_) {
    return fit_spline(config_.precision_, config_.order_, config_.coeffs_ratio_, unordered_path);
  } else {
    return unordered_path;
  }
}