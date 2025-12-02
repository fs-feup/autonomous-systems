#include "slam_solver/graph_slam_solver/optimizer/normal_levenberg_optimizer.hpp"

NormalLevenbergOptimizer::NormalLevenbergOptimizer(const SLAMParameters& params)
    : BaseOptimizer(params){};

std::shared_ptr<BaseOptimizer> NormalLevenbergOptimizer::clone() const {
  return std::make_shared<NormalLevenbergOptimizer>(*this);
}

gtsam::Values NormalLevenbergOptimizer::optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                                 gtsam::Values& graph_values,
                                                 [[maybe_unused]] unsigned int pose_num,
                                                 [[maybe_unused]] unsigned int landmark_num) {
  gtsam::LevenbergMarquardtOptimizer optimizer(factor_graph, graph_values);
  return optimizer.optimize();
}