#include "slam_solver/graph_slam_solver.hpp"

#include <gtest/gtest.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

TEST(GraphSLAMSolver, Test) {
  // Arrange
  gtsam::NonlinearFactorGraph graph;

  static gtsam::Symbol x1('x', 1), x2('x', 2), x3('x', 3);
  static gtsam::Symbol l1('l', 1), l2('l', 2), l3('l', 3);

  // Act

  // Assert
  ASSERT_TRUE(true);
}