#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"

/**
 * @brief Sliding Window Levenberg-Marquardt optimizer for graph SLAM
 * @details This class implements the sliding window Levenberg-Marquardt optimizer for graph SLAM
 * It uses the GTSAM Levenberg-Marquardt implementation to optimize the factor graph
 * within a sliding window of poses
 */
class SlidingWindowLevenbergOptimizer : public BaseOptimizer {
public:
  SlidingWindowLevenbergOptimizer(const SLAMParameters& params);

  ~SlidingWindowLevenbergOptimizer() override = default;

  /**
   * @brief Clone the optimizer
   * @details This method is used to create a copy of the optimizer
   * It is useful for polymorphic classes that use pointers to base class
   *
   * @return A shared pointer to the cloned optimizer
   */
  std::shared_ptr<BaseOptimizer> clone() const override;

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
  gtsam::Values optimize(gtsam::NonlinearFactorGraph& factor_graph, gtsam::Values& graph_values,
                         [[maybe_unused]] unsigned int pose_num,
                         [[maybe_unused]] unsigned int landmark_num) override;
};