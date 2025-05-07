#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"

class NormalLevenbergOptimizer : public BaseOptimizer {
public:
  NormalLevenbergOptimizer(const SLAMParameters& params);

  ~NormalLevenbergOptimizer() override = default;

  gtsam::Values optimize(gtsam::NonlinearFactorGraph& factor_graph, gtsam::Values& graph_values,
                         [[maybe_unused]] unsigned int pose_num,
                         [[maybe_unused]] unsigned int landmark_num) override;
};