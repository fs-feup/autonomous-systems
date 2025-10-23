#include "slam_solver/graph_slam_solver/optimizer/sliding_window_levenberg_optimizer.hpp"

#include <gtsam/base/timing.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/SparseCore>

std::shared_ptr<BaseOptimizer> SlidingWindowLevenbergOptimizer::clone() const {
  return std::make_shared<SlidingWindowLevenbergOptimizer>(*this);
}

SlidingWindowLevenbergOptimizer::SlidingWindowLevenbergOptimizer(const SLAMParameters& params)
    : BaseOptimizer(params){};

gtsam::Values SlidingWindowLevenbergOptimizer::optimize(
    gtsam::NonlinearFactorGraph& factor_graph, gtsam::Values& graph_values,
    [[maybe_unused]] unsigned int pose_num, [[maybe_unused]] unsigned int landmark_num) {
  // Retrieve the last poses added to the graph
  std::set<gtsam::Key> pose_keys;
  std::set<gtsam::Key> landmark_keys;
  gtsam::NonlinearFactorGraph sliding_window_graph;
  gtsam::Values sliding_window_values;
  for (int i = std::max<int>(1, pose_num - _params_.sliding_window_size_ + 1);
       i <= static_cast<int>(pose_num); ++i) {
    pose_keys.insert(gtsam::Symbol('x', i));
  }

  // Add relevant factors
  for (const auto& factor : factor_graph) {
    auto involved_keys = factor->keys();
    bool include = false;
    for (const auto& key : involved_keys) {
      if (gtsam::Symbol(key).chr() == 'x' && pose_keys.count(key) == 0) {
        // If one of the involved nodes in the factor is a pose
        // and not one in the sliding window, ignore the factor
        include = false;
        break;
      } else if (gtsam::Symbol(key).chr() == 'x' && pose_keys.count(key) > 0) {
        // If one of the involved nodes in the factor is a pose
        // and is in the sliding window, include the factor, but
        // wait to check if the other node is a pose and not in the
        // sliding window
        include = true;
      }
    }
    if (include) {
      sliding_window_graph.add(factor);
    }
  }

  // Determine relevant landmarks through factors
  for (const auto& factor : sliding_window_graph) {
    auto involved_keys = factor->keys();
    for (const auto& key : involved_keys) {
      if (gtsam::Symbol(key).chr() == 'l') {
        landmark_keys.insert(key);
      }
    }
  }

  // Add prior factors for included landmarks only
  for (const auto& factor : factor_graph) {
    const auto& keys = factor->keys();

    // Check if this is a prior factor on a landmark in the current window
    if (keys.size() == 1) {
      gtsam::Symbol symbol(keys[0]);
      if (symbol.chr() == 'l' && landmark_keys.count(keys[0]) > 0) {
        sliding_window_graph.add(factor);
      }
    }
  }

  // Add relevant values
  for (const auto& key : landmark_keys) {
    sliding_window_values.insert(key, graph_values.at(key));
  }
  for (const auto& key : pose_keys) {
    sliding_window_values.insert(key, graph_values.at(key));
  }

  // Define parameters
  gtsam::LevenbergMarquardtParams lm_params;
  gtsam::Ordering ordering;
  for (const auto& key : landmark_keys) {
    ordering.push_back(key);
  }
  for (const auto& key : pose_keys) {
    ordering.push_back(key);
  }
  lm_params.setOrdering(ordering);
  // lm_params.setMaxIterations(2);       // steady latency
  // lm_params.setlambdaInitial(1e-3);    // sane start
  lm_params.setlambdaUpperBound(1e3);  // clamp runaway damping
  // lm_params.setlambdaFactor(10.0);     // backoff multiplier
  // lm_params.setAbsoluteErrorTol(0);    // disable early-stopping by tolerance (latency focus)
  // lm_params.setRelativeErrorTol(0);    // "
  // lm_params.setLinearSolverType(gtsam::LevenbergMarquardtParams::MULTIFRONTAL_CHOLESKY);
  // lm_params.setOrderingType(
  //     gtsam::LevenbergMarquardtParams::OrderingType::CUSTOM);  // supply your ordering
  // TODO: determine best parameters and expose them to YAML

  // Optimize the sliding window
  rclcpp::Time start_time = rclcpp::Clock().now();
  gtsam::LevenbergMarquardtOptimizer optimizer(sliding_window_graph, sliding_window_values,
                                               lm_params);
  graph_values.update(optimizer.optimize());
  rclcpp::Duration optimization_time = rclcpp::Clock().now() - start_time;
  RCLCPP_DEBUG(
      rclcpp::get_logger("slam"),
      "SlidingWindowLevenbergOptimizer - Optimized %d poses and %d landmarks, including %d factors",
      static_cast<int>(pose_keys.size()), static_cast<int>(landmark_keys.size()),
      static_cast<int>(sliding_window_graph.size()));
  RCLCPP_DEBUG(rclcpp::get_logger("slam"),
               "SlidingWindowLevenbergOptimizer - Lambda at start: %lf, end: %lf",
               lm_params.lambdaInitial, optimizer.lambda());
  RCLCPP_DEBUG(rclcpp::get_logger("slam"),
               "SlidingWindowLevenbergOptimizer - Number of iterations: "
               "%ld\nOptimization  time: %f",
               optimizer.iterations(), optimization_time.seconds());
  return graph_values;
}