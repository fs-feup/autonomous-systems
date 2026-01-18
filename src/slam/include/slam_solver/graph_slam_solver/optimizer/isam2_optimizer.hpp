#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"

/**
 * @brief ISAM2 optimizer for graph SLAM
 * @details This class implements the ISAM2 optimizer for graph SLAM
 * It uses the GTSAM ISAM2 implementation to optimize the factor graph incrementally
 */
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

  /**
   * @brief Clone the optimizer
   * @details This method is used to create a copy of the optimizer
   * It is useful for polymorphic classes that use pointers to base class
   *
   * @return A shared pointer to the cloned optimizer
   */
  std::shared_ptr<BaseOptimizer> clone() const;

  /**
   * @brief Optimize the graph
   * @details This method is used to run the optimization on the graph
   * It also updates the pose and the graph values accordingly afterwards
   * It may change the factor graph
   * ISAM2 optimization is incremental, so it uses its own graph and values to optimize
   * TODO: improve this architecture to avoid confusion
   *
   * @param factor_graph The factor graph to optimize
   * @param graph_values The values to optimize
   * @param pose_num The number of poses in the graph
   * @param landmark_num The number of landmarks in the graph
   * @return The optimized values
   */
  gtsam::Values optimize([[maybe_unused]] gtsam::NonlinearFactorGraph& factor_graph,
                         [[maybe_unused]] gtsam::Values& graph_values,
                         [[maybe_unused]] unsigned int pose_num,
                         [[maybe_unused]] unsigned int landmark_num) override;

  friend class GraphSLAMInstance;
};