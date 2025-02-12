#include "slam_solver/graph_slam_solver.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

GraphSLAMSolver::GraphSLAMSolver() : SLAMSolver() {
  // Create a new factor graph
  _factor_graph_ = gtsam::NonlinearFactorGraph();
  gtsam::Pose2 prior_pose(0.0, 0.0, 0.0);
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.0, 0.0, 0.0));
  _factor_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(1, prior_pose, prior_noise));
}

void GraphSLAMSolver::add_motion_prior(const common_lib::structures::Velocities& velocities) {
  // Create a new prior factor
  // Add the prior factor to the graph
}

void GraphSLAMSolver::add_observation(const common_lib::structures::Position& position) {
  // Create a new observation factor
  // Add the observation factor to the graph
}

// void GraphSLAMSolver::solve() {
//   // Create a new optimizer
//   gtsam::LevenbergMarquardtOptimizer optimizer(graph, gtsam::Pose2(0.0, 0.0, 0.0));
//   // Optimize the graph
//   Values result = optimizer.optimize();
//   // Get the optimized pose
//   gtsam::Pose2 optimized_pose = result.at<gtsam::Pose2>(X(0));
//   // Get the optimized landmark
//   gtsam::Point2 optimized_landmark = result.at<gtsam::Point2>(L(0));
//   // Print the optimized pose and landmark
//   std::cout << "Optimized pose: " << optimized_pose << std::endl;
//   std::cout << "Optimized landmark: " << optimized_landmark << std::endl;
// }

// std::vector<common_lib::structures::Cone> GraphSLAMSolver::get_map_estimate() {
//   // Create a vector to store the map estimate
//   std::vector<common_lib::structures::Cone> map_estimate;
//   // Get the optimized landmark
//   gtsam::Point2 optimized_landmark = result.at<gtsam::Point2>(L(0));
//   // Create a new cone with the optimized landmark
//   common_lib::structures::Cone cone(optimized_landmark.x(), optimized_landmark.y(), 0.0);
//   // Add the cone to the map estimate
//   map_estimate.push_back(cone);
//   // Return the map estimate
//   return map_estimate;
// }

// common_lib::structures::Pose GraphSLAMSolver::get_pose_estimate() {
//   // Get the optimized pose
//   gtsam::Pose2 optimized_pose = result.at<gtsam::Pose2>(X(0));
//   // Create a new pose with the optimized pose
//   common_lib::structures::Pose pose(optimized_pose.x(), optimized_pose.y(),
//   optimized_pose.theta());
//   // Return the pose estimate
//   return pose;
// }