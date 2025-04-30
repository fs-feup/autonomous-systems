#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

class BaseOptimizer {
public:
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
                                 gtsam::Values& graph_values) = 0;
};