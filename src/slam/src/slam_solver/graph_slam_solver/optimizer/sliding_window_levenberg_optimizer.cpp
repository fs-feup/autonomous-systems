#include "slam_solver/graph_slam_solver/optimizer/sliding_window_levenberg_optimizer.hpp"

#include <gtsam/inference/Symbol.h>

SlidingWindowLevenbergOptimizer::SlidingWindowLevenbergOptimizer(const SLAMParameters& params)
    : BaseOptimizer(params){};

gtsam::Values SlidingWindowLevenbergOptimizer::optimize(
    gtsam::NonlinearFactorGraph& factor_graph, gtsam::Values& graph_values,
    [[maybe_unused]] unsigned int pose_num, [[maybe_unused]] unsigned int landmark_num) {
  // Retrieve the last 5 poses added to the graph
  std::unordered_set<gtsam::Key> relevant_keys;
  gtsam::NonlinearFactorGraph sliding_window_graph;
  gtsam::Values sliding_window_values;
  for (int i = std::max<int>(1, pose_num - _params_.sliding_window_size_ + 1);
       i <= static_cast<int>(pose_num); ++i) {
    relevant_keys.insert(gtsam::Symbol('x', i));
  }

  // Add relevant factors
  for (const auto& factor : factor_graph) {
    auto involved_keys = factor->keys();
    bool include = false;
    for (const auto& key : involved_keys) {
      if (gtsam::Symbol(key).chr() == 'x' && relevant_keys.count(key) == 0) {
        // If one of the involved nodes in the factor is a pose
        // and not one in the sliding window, ignore the factor
        include = false;
        break;
      } else if (gtsam::Symbol(key).chr() == 'x' && relevant_keys.count(key) > 0) {
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
        relevant_keys.insert(key);
      }
    }
  }

  // Add relevant values
  for (const auto& key : relevant_keys) {
    sliding_window_values.insert(key, graph_values.at(key));
  }

  // Optimize the sliding window
  gtsam::LevenbergMarquardtOptimizer optimizer(sliding_window_graph, sliding_window_values);
  graph_values.update(optimizer.optimize());
  return graph_values;
}