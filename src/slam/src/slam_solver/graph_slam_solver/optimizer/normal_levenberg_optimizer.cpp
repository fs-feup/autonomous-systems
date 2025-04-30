#include "slam_solver/graph_slam_solver/optimizer/normal_levenberg_optimizer.hpp"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

gtsam::Values NormalLevenbergOptimizer::optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                                 gtsam::Values& graph_values) {
  gtsam::LevenbergMarquardtOptimizer optimizer(factor_graph, graph_values);
  return optimizer.optimize();
}