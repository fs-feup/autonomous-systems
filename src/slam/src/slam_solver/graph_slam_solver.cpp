#include "slam_solver/graph_slam_solver.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "common_lib/maths/transformations.hpp"

GraphSLAMSolver::GraphSLAMSolver(const SLAMSolverParameters& params,
                                 std::shared_ptr<DataAssociationModel> data_association,
                                 std::shared_ptr<V2PMotionModel> motion_model)
    : SLAMSolver(params, data_association, motion_model) {
  // Create a new factor graph
  _factor_graph_ = gtsam::NonlinearFactorGraph();
  const gtsam::Pose2 prior_pose(0.0, 0.0, 0.0);
  const gtsam::Symbol pose_symbol('x', ++(this->_pose_counter_));
  const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.0, 0.0, 0.0));
  _factor_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(pose_symbol, prior_pose, prior_noise));

  // Create a new values object
  _graph_values_ = gtsam::Values();
  _graph_values_.insert(pose_symbol, prior_pose);
}

void GraphSLAMSolver::add_motion_prior(const common_lib::structures::Velocities& velocities) {
  if (_last_pose_update_ == rclcpp::Time(0)) {
    return;
  }

  // Apply Motion Model
  const double delta_t =
      (velocities.timestamp - this->_last_pose_update_).seconds();  // Get the time

  const Eigen::Vector3d velocities_vector(velocities.velocity_x, velocities.velocity_y,
                                          velocities.rotational_velocity);
  const gtsam::Pose2 last_pose =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  const Eigen::Vector3d previous_pose(last_pose.x(), last_pose.y(), last_pose.theta());
  const Eigen::Vector3d next_pose =
      this->_motion_model_->get_next_pose(previous_pose, velocities_vector, delta_t);

  // Create a new pose
  gtsam::Pose2 new_pose(next_pose[0], next_pose[1], next_pose[2]);
  gtsam::Symbol previous_pose_symbol('x', this->_pose_counter_);
  gtsam::Symbol new_pose_symbol('x', ++(this->_pose_counter_));

  // Add the prior factor to the graph
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.1, 0.1, 0.1));  // TODO(marhcouto): new noise model for each motion model

  _factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previous_pose_symbol, new_pose_symbol,
                                                        new_pose, prior_noise));

  // Add the prior pose to the values
  _graph_values_.insert(new_pose_symbol, new_pose);

  // Update the last pose update
  _last_pose_update_ = velocities.timestamp;
}

void GraphSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
  // Prepare structures for data association
  Eigen::VectorXd observations(cones.size() * 2);
  Eigen::VectorXd observations_confidences(cones.size());
  Eigen::VectorXd landmarks(this->_landmark_counter_ * 2);

  unsigned int landmark_id = 0;
  for (auto it = _graph_values_.begin(); it != _graph_values_.end(); ++it) {
    // Iterate through the landmarks in the _graph_values_
    if (gtsam::Symbol(it->key).chr() == 'l') {
      gtsam::Point2 landmark = it->value.cast<gtsam::Point2>();
      landmarks(landmark_id++) = landmark.x();
      landmarks(landmark_id++) = landmark.y();
    }
  }

  for (unsigned int i = 0; i < cones.size(); i++) {
    // Iterate through the cones
    observations(i * 2) = cones[i].position.x;
    observations(i * 2 + 1) = cones[i].position.y;
    observations_confidences(i) = cones[i].certainty;
  }

  // Perform Data Association
  // Eigen::VectorXd associations(cones.size()) = this->_data_association_->associate_data(
  //     observations, observations_confidences, landmarks, this->_params_.observation_x_noise_,
  //     this->_params_.observation_y_noise_);
  Eigen::VectorXd associations(cones.size());
  for (unsigned int i = 0; i < cones.size(); i++) {
    associations(i) = -1;
  }

  // Create a new observation factor
  for (unsigned int i = 0; i < cones.size(); i++) {
    gtsam::Point2 landmark;
    gtsam::Symbol landmark_symbol;
    if (associations(i) == -1) {
      // Create new landmark
      landmark_symbol = gtsam::Symbol('l', ++(this->_landmark_counter_));
      landmark = gtsam::Point2(observations(i * 2), observations(i * 2 + 1));
      _graph_values_.insert(landmark_symbol, landmark);
    } else {
      // Association to previous landmark
      landmark_symbol = gtsam::Symbol('l', associations(i));
      landmark = _graph_values_.at<gtsam::Point2>(landmark_symbol);
    }
    Eigen::Vector2d landmark_cartesian(landmark.x(), landmark.y());
    Eigen::Vector2d landmark_cylindrical = common_lib::maths::cartesian_to_cylindrical(
        landmark_cartesian);  // Convert to cylindrical coordinates
    gtsam::Rot2 landmark_rotation(landmark_cylindrical(1));

    // Add the observation factor to the graph
    this->_factor_graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
        gtsam::Symbol('x', this->_pose_counter_), landmark_symbol, landmark_rotation,
        landmark_cylindrical(0),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(this->_params_.observation_x_noise_,
                                                           this->_params_.observation_x_noise_))));
  }
}

void GraphSLAMSolver::optimize() {
  gtsam::LevenbergMarquardtOptimizer optimizer(this->_factor_graph_, this->_graph_values_);
  this->_graph_values_ = optimizer.optimize();
}

std::vector<common_lib::structures::Cone> GraphSLAMSolver::get_map_estimate() {
  rclcpp::Time current_time = rclcpp::Clock().now();
  std::vector<common_lib::structures::Cone> map_estimate;
  for (auto it = _graph_values_.begin(); it != _graph_values_.end(); ++it) {
    // Iterate through the landmarks in the _graph_values_
    if (gtsam::Symbol(it->key).chr() == 'l') {
      gtsam::Point2 landmark = it->value.cast<gtsam::Point2>();
      map_estimate.push_back(
          common_lib::structures::Cone(landmark.x(), landmark.y(), "unknown", 1.0, current_time));
    }
  }
  return map_estimate;
}

common_lib::structures::Pose GraphSLAMSolver::get_pose_estimate() {
  gtsam::Pose2 last_pose =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  return common_lib::structures::Pose(last_pose.x(), last_pose.y(), last_pose.theta());
}