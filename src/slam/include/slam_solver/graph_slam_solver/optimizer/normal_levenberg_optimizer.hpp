#pragma once

#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"

class NormalLevenbergOptimizer : public BaseOptimizer {
public:
  NormalLevenbergOptimizer() = default;

  ~NormalLevenbergOptimizer() override = default;

  gtsam::Values optimize(gtsam::NonlinearFactorGraph& factor_graph,
                         gtsam::Values& graph_values) override;
};