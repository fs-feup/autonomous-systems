#include "slam_solver/graph_slam_solver/optimizer/isam2_optimizer.hpp"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <rclcpp/rclcpp.hpp>

ISAM2Optimizer::ISAM2Optimizer(const SLAMParameters& params) : BaseOptimizer(params) {
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = params.slam_isam2_relinearize_threshold_;
  isam_params.relinearizeSkip = params.slam_isam2_relinearize_skip_;
  isam_params.factorization = params.slam_isam2_factorization_ == "QR"
                                  ? gtsam::ISAM2Params::QR
                                  : gtsam::ISAM2Params::CHOLESKY;
  _isam2_ = gtsam::ISAM2(isam_params);
  _last_estimate_ = gtsam::Values();
};

gtsam::Values ISAM2Optimizer::optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                       gtsam::Values& graph_values,
                                       [[maybe_unused]] unsigned int pose_num,
                                       [[maybe_unused]] unsigned int landmark_num) {
  // Remove values from graph_values that are in _last_estimate_
  gtsam::Values new_values;
  for (const auto& key : graph_values.keys()) {
    if (!_last_estimate_.exists(key)) {
      new_values.insert(key, graph_values.at(key));
    }
  }

  _isam2_.update(factor_graph, new_values);
  factor_graph.resize(0);  // Clear the factor graph
  _last_estimate_ = _isam2_.calculateEstimate();
  return _last_estimate_;
}