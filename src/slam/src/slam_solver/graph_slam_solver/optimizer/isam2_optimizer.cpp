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

ISAM2Optimizer::ISAM2Optimizer(const ISAM2Optimizer& other) : BaseOptimizer(other) {
  this->_isam2_ = other._isam2_;
  this->_last_estimate_ = other._last_estimate_;
  this->_new_values_ = other._new_values_;
  this->_new_factors_ = other._new_factors_;
}

ISAM2Optimizer& ISAM2Optimizer::operator=(const ISAM2Optimizer& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  BaseOptimizer::operator=(other);
  this->_isam2_ = other._isam2_;
  this->_last_estimate_ = other._last_estimate_;
  this->_new_values_ = other._new_values_;
  this->_new_factors_ = other._new_factors_;

  return *this;
}

std::shared_ptr<BaseOptimizer> ISAM2Optimizer::clone() const {
  return std::make_shared<ISAM2Optimizer>(*this);
}

gtsam::Values ISAM2Optimizer::optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                       gtsam::Values& graph_values,
                                       [[maybe_unused]] unsigned int pose_num,
                                       [[maybe_unused]] unsigned int landmark_num) {
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "ISAM2Optimizer - Adding %zu new factors",
               factor_graph.size());

  // Ensure all keys in factor_graph exist in either _last_estimate_ or _new_values_ DONT KNOW IF
  // CHANGES ANYTHING
  for (const auto& factor : factor_graph) {
    gtsam::KeyVector keys = factor->keys();  // use KeyVector directly
    for (gtsam::Key key : keys) {
      if (!_last_estimate_.exists(key) && !_new_values_.exists(key)) {
        // Insert an initial guess
        if (gtsam::Symbol(key).chr() == 'x') {
          _new_values_.insert(key, gtsam::Pose2(0, 0, 0));
        } else if (gtsam::Symbol(key).chr() == 'l') {
          _new_values_.insert(key, gtsam::Point2(0, 0));
        }
      }
    }
  }

  // Add new factors and values to iSAM2
  _isam2_.update(this->_new_factors_, this->_new_values_);
  _new_factors_.resize(0);
  _new_values_.clear();

  _last_estimate_ = _isam2_.calculateEstimate();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "ISAM2Optimizer - Optimization complete");
  return _last_estimate_;
}
