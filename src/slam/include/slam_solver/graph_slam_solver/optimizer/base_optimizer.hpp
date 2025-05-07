#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "slam_config/general_config.hpp"

class BaseOptimizer {
protected:
  const SLAMParameters& _params_;

public:
  explicit BaseOptimizer(const SLAMParameters& params) : _params_(params){};
  virtual ~BaseOptimizer() = default;

  /**
   * @brief Optimize the graph
   * @details This method is used to run the optimization on the graph
   * It also updates the pose and the graph values accordingly afterwards
   * It may change the factor graph
   *
   * @param factor_graph The factor graph to optimize
   * @param graph_values The values to optimize
   * @return The optimized values
   */
  virtual gtsam::Values optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                 gtsam::Values& graph_values,
                                 [[maybe_unused]] unsigned int pose_num,
                                 [[maybe_unused]] unsigned int landmark_num) = 0;
};