#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"

class ISAM2Optimizer : public BaseOptimizer {
  gtsam::ISAM2 _isam2_;                       //< ISAM2 instance for the optimizer
  gtsam::Values _last_estimate_;              //< Last estimate from the optimizer
  gtsam::Values _new_values_;                 //< New values to add to the optimizer
  gtsam::NonlinearFactorGraph _new_factors_;  //< New factors to add to the optimizer
public:
  ISAM2Optimizer(const SLAMParameters& params);
  ISAM2Optimizer(const ISAM2Optimizer& other);
  ISAM2Optimizer& operator=(const ISAM2Optimizer& other);

  ~ISAM2Optimizer() override = default;

  gtsam::Values optimize(gtsam::NonlinearFactorGraph& factor_graph, gtsam::Values& graph_values,
                         [[maybe_unused]] unsigned int pose_num,
                         [[maybe_unused]] unsigned int landmark_num) override;

  friend class GraphSLAMInstance;
};