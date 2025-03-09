#include "slam_solver/graph_slam_solver.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "common_lib/maths/transformations.hpp"

GraphSLAMSolver::GraphSLAMSolver(const SLAMParameters& params,
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
    _last_pose_update_ = velocities.timestamp;
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
  gtsam::Pose2 pose_difference(next_pose[0] - previous_pose[0], next_pose[1] - previous_pose[1],
                               next_pose[2] - previous_pose[2]);
  gtsam::Symbol previous_pose_symbol('x', this->_pose_counter_);
  gtsam::Symbol new_pose_symbol('x', ++(this->_pose_counter_));

  // Add the prior factor to the graph
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.1, 0.1, 0.1));  // TODO(marhcouto): new noise model for each motion model

  _factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previous_pose_symbol, new_pose_symbol,
                                                        pose_difference, prior_noise));

  // Add the prior pose to the values
  _graph_values_.insert(new_pose_symbol, new_pose);

  // Update the last pose update
  _last_pose_update_ = velocities.timestamp;
}

void GraphSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
  // Prepare structures for data association
  Eigen::VectorXd observations(cones.size() * 2);
  Eigen::VectorXd observations_confidences(cones.size());
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "landmark_counter: %d", this->_landmark_counter_);
  Eigen::VectorXd state(this->_landmark_counter_ * 2 + 3);
  gtsam::Pose2 current_pose =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  state(0) = current_pose.x();
  state(1) = current_pose.y();
  state(2) = current_pose.theta();
  unsigned int landmark_id = 3;
  for (auto it = _graph_values_.begin(); it != _graph_values_.end(); ++it) {
    // Iterate through the landmarks in the _graph_values_
    if (gtsam::Symbol(it->key).chr() == 'l') {
      gtsam::Point2 landmark = it->value.cast<gtsam::Point2>();
      state(landmark_id++) = landmark.x();
      state(landmark_id++) = landmark.y();
    }
  }

  for (unsigned int i = 0; i < cones.size(); i++) {
    // Iterate through the cones
    observations(i * 2) = cones[i].position.x;
    observations(i * 2 + 1) = cones[i].position.y;
    observations_confidences(i) = cones[i].certainty;
  }

  Eigen::VectorXd observations_global = common_lib::maths::local_to_global_coordinates(
      state.head<3>(), observations);  // Convert to global coordinates

  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "I am here 1");
  // Data association
  Eigen::VectorXi associations(cones.size());
  associations = this->_data_association_->associate(state, this->_get_covariance(), observations,
                                                     observations_confidences);

  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "I am here 2");

  // Create a new observation factor
  for (unsigned int i = 0; i < cones.size(); i++) {
    gtsam::Point2 landmark;
    gtsam::Symbol landmark_symbol;
    if (associations(i) == -2) {
      // No association
      continue;
    } else if (associations(i) == -1) {
      // Create new landmark
      landmark_symbol = gtsam::Symbol('l', ++(this->_landmark_counter_));
      landmark = gtsam::Point2(observations_global(i * 2), observations_global(i * 2 + 1));
      _graph_values_.insert(landmark_symbol, landmark);
    } else {
      // Association to previous landmark
      landmark_symbol = gtsam::Symbol('l', associations(i));
    }
    const Eigen::Vector2d observation_cartesian(cones[i].position.x, cones[i].position.y);
    Eigen::Vector2d observation_cylindrical = common_lib::maths::cartesian_to_cylindrical(
        observation_cartesian);  // Convert to cylindrical coordinates
    const gtsam::Rot2 observation_rotation(observation_cylindrical(1));

    const gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(this->_params_.observation_x_noise_,
                                                           this->_params_.observation_y_noise_));

    // Add the observation factor to the graph
    this->_factor_graph_.add(gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2>(
        gtsam::Symbol('x', this->_pose_counter_), landmark_symbol, observation_rotation,
        gtsam::noiseModel::Isotropic::Sigma(1, this->_params_.observation_x_noise_)));
    this->_factor_graph_.add(gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>(
        gtsam::Symbol('x', this->_pose_counter_), landmark_symbol, observation_cylindrical(0),
        gtsam::noiseModel::Isotropic::Sigma(1, this->_params_.observation_x_noise_)));

    // this->_factor_graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
    //     gtsam::Symbol('x', this->_pose_counter_), landmark_symbol, observation_rotation,
    //     observation_cylindrical(0), observation_noise));
  }

  // Optimize the graph
  this->_optimize();
}

void GraphSLAMSolver::_optimize() {
  gtsam::LevenbergMarquardtOptimizer optimizer(this->_factor_graph_, this->_graph_values_);
  this->_graph_values_ = optimizer.optimize();
}

Eigen::MatrixXd GraphSLAMSolver::_get_covariance() {
  // gtsam::Ordering ordering = gtsam::Ordering();
  // ordering.push_back(gtsam::Symbol('x', this->_pose_counter_));
  // for (unsigned int i = 1; i <= this->_landmark_counter_; i++) {
  //   ordering.push_back(gtsam::Symbol('l', i));
  // }
  // const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_, ordering);

  // // Create a gtsam::KeyVector with keys of all landmarks and the current pose, current pose
  // first,
  // // landmarks ordered by id
  // gtsam::KeyVector keys;
  // keys.push_back(gtsam::Symbol('x', this->_pose_counter_));
  // for (unsigned int i = 1; i <= this->_landmark_counter_; i++) {
  //   keys.push_back(gtsam::Symbol('l', i));
  // }

  // Eigen::MatrixXd covariance = marginals.jointMarginalCovariance(keys).fullMatrix();

  // return covariance;

  return Eigen::MatrixXd();  // Temporary
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