#include "slam_solver/graph_slam_solver/optimizer/isam2_optimizer.hpp"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>

ISAM2Optimizer::ISAM2Optimizer() {
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_params.enableDetailedResults = false;
  _isam2_ = gtsam::ISAM2(isam_params);
  _last_estimate_ = gtsam::Values();
};

gtsam::Values ISAM2Optimizer::optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                       gtsam::Values& graph_values) {
  // Remove values from graph_values that are in _last_estimate_
  for (const auto& key : _last_estimate_.keys()) {
    if (graph_values.exists(key)) {
      graph_values.erase(key);
    }
  }
  _isam2_.update(factor_graph, graph_values);
  factor_graph.resize(0);  // Clear the factor graph
  _last_estimate_ = _isam2_.calculateEstimate();
  return _last_estimate_;
}