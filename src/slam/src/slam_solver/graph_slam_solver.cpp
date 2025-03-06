#include "slam_solver/graph_slam_solver.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

GraphSLAMSolver::GraphSLAMSolver(const SLAMSolverParameters& params,
                                 std::shared_ptr<DataAssociationModel> data_association,
                                 std::shared_ptr<V2PMotionModel> motion_model)
    : SLAMSolver(params, data_association, motion_model) {
  // Create a new factor graph
  _factor_graph_ = gtsam::NonlinearFactorGraph();
  gtsam::Pose2 prior_pose(0.0, 0.0, 0.0);
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.0, 0.0, 0.0));
  _factor_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(1, prior_pose, prior_noise));
  // Create a new values object
}

void GraphSLAMSolver::add_motion_prior(const common_lib::structures::Velocities& velocities) {
  if (_last_pose_update_ == rclcpp::Time(0)) {
    return;
  }
  // Prepare data
  double delta_t = (velocities.timestamp - this->_last_pose_update_).seconds();  // Get the time

  Eigen::Vector3d velocities_vector(velocities.velocity_x, velocities.velocity_y,
                                    velocities.rotational_velocity);
  gtsam::Pose2 last_pose =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  Eigen::Vector3d previous_pose(last_pose.x(), last_pose.y(), last_pose.theta());
  Eigen::Vector3d next_pose =
      this->_motion_model_->get_next_pose(previous_pose, velocities_vector, delta_t);
  gtsam::Pose2 prior_pose(next_pose[0], next_pose[1], next_pose[2]);

  // Add the prior factor to the graph
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.1, 0.1, 0.1));  // TODO(marhcouto): new noise model for each motion model

  _factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 2, prior_pose, prior_noise));

  // Add the prior pose to the values
  gtsam::Symbol pose_symbol('x', ++(this->_pose_counter_));
  _graph_values_.insert(pose_symbol, prior_pose);

  // Update the last pose update
  _last_pose_update_ = velocities.timestamp;
}

void GraphSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
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

std::vector<common_lib::structures::Cone> GraphSLAMSolver::get_map_estimate() {
  // // Create a vector to store the map estimate
  // std::vector<common_lib::structures::Cone> map_estimate;
  // // Get the optimized landmark
  // gtsam::Point2 optimized_landmark = result.at<gtsam::Point2>(L(0));
  // // Create a new cone with the optimized landmark
  // common_lib::structures::Cone cone(optimized_landmark.x(), optimized_landmark.y(), 0.0);
  // // Add the cone to the map estimate
  // map_estimate.push_back(cone);
  // // Return the map estimate
  // return map_estimate;
}

common_lib::structures::Pose GraphSLAMSolver::get_pose_estimate() {
  gtsam::Pose2 last_pose =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  return common_lib::structures::Pose(last_pose.x(), last_pose.y(), last_pose.theta());
  // // Get the optimized pose
  // gtsam::Pose2 optimized_pose = result.at<gtsam::Pose2>(X(0));
  // // Create a new pose with the optimized pose
  // common_lib::structures::Pose pose(optimized_pose.x(), optimized_pose.y(),
  // optimized_pose.theta());
  // // Return the pose estimate
  // return pose;
}