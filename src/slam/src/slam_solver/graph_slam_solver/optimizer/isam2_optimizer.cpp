#include "slam_solver/graph_slam_solver/optimizer/isam2_optimizer.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unordered_set>

// ----------------- Usage -----------------
/*
FactorSet tracker;
gtsam::NonlinearFactorGraph newFactors;

auto odom = std::make_shared<gtsam::BetweenFactor<gtsam::Pose2>>(X(1), X(2),
gtsam::Pose2(1.0,0.0,0.0), noise); if (!tracker.alreadyAdded(odom)) { newFactors.add(odom);
tracker.add(odom); }

auto br = std::make_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>(X(2), L(5),
gtsam::Rot2::fromAngle(0.12), 7.34, noise); if (!tracker.alreadyAdded(br)) { newFactors.add(br);
tracker.add(br); }
*/

ISAM2Optimizer::ISAM2Optimizer(const SLAMParameters& params) : BaseOptimizer(params) {
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = params.slam_isam2_relinearize_threshold_;
  isam_params.relinearizeSkip = params.slam_isam2_relinearize_skip_;
  isam_params.factorization = params.slam_isam2_factorization_ == "QR"
                                  ? gtsam::ISAM2Params::QR
                                  : gtsam::ISAM2Params::CHOLESKY;
  _isam2_ = gtsam::ISAM2(isam_params);
  _last_estimate_ = gtsam::Values();
  _new_factors_ = gtsam::NonlinearFactorGraph();
  _new_values_ = gtsam::Values();
};

gtsam::Values ISAM2Optimizer::optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                       gtsam::Values& graph_values,
                                       [[maybe_unused]] unsigned int pose_num,
                                       [[maybe_unused]] unsigned int landmark_num) {
  // Remove values from graph_values that are in _last_estimate_
  _isam2_.update(this->_new_factors_, this->_new_values_);
  this->_new_factors_.resize(0);
  this->_new_values_.clear();
  _last_estimate_ = _isam2_.calculateEstimate();
  return _last_estimate_;
}