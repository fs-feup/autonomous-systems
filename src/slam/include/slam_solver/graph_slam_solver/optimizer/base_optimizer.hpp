#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "slam_config/general_config.hpp"

/**
 * @brief Base class for graph optimizers
 * @details This class defines the interface for graph optimizers
 * It is used to optimize the factor graph and update the pose and graph values accordingly
 */
class BaseOptimizer {
protected:
  const SLAMParameters& _params_;

public:
  explicit BaseOptimizer(const SLAMParameters& params) : _params_(params){};
  BaseOptimizer(const BaseOptimizer& other) : _params_(other._params_){};
  BaseOptimizer& operator=(const BaseOptimizer& other) {
    if (this == &other) return *this;  // Prevent self-assignment

    // Note: _params_ is a reference, so we do not copy it
    return *this;
  }

  /**
   * @brief Clone the optimizer
   * @details This method is used to create a copy of the optimizer
   * It is useful for polymorphic classes that use pointers to base class
   *
   * @return A shared pointer to the cloned optimizer
   */
  virtual std::shared_ptr<BaseOptimizer> clone() const = 0;

  virtual ~BaseOptimizer() = default;

  /**
   * @brief Optimize the graph
   * @details This method is used to run the optimization on the graph
   * It also updates the pose and the graph values accordingly afterwards
   * It may change the factor graph
   *
   * @param factor_graph The factor graph to optimize
   * @param graph_values The values to optimize
   * @param pose_num The number of poses in the graph
   * @param landmark_num The number of landmarks in the graph
   * @return The optimized values
   */
  virtual gtsam::Values optimize(gtsam::NonlinearFactorGraph& factor_graph,
                                 gtsam::Values& graph_values,
                                 [[maybe_unused]] unsigned int pose_num,
                                 [[maybe_unused]] unsigned int landmark_num) = 0;
};