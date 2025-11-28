#include "slam_solver/graph_slam_solver/graph_slam_instance.hpp"

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
#include "slam_solver/graph_slam_solver/optimizer/isam2_optimizer.hpp"
#include "slam_solver/graph_slam_solver/utils.hpp"

bool GraphSLAMInstance::new_pose_factors() const { return this->_new_pose_node_; }

bool GraphSLAMInstance::new_observation_factors() const { return this->_new_observation_factors_; }

const gtsam::Values& GraphSLAMInstance::get_graph_values_reference() const {
  return this->_graph_values_;
}

const gtsam::Pose2 GraphSLAMInstance::get_pose() const {
  return this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
}

Eigen::VectorXd GraphSLAMInstance::get_state_vector() const {
  Eigen::VectorXd state(this->_landmark_counter_ * 2 + 3);
  const gtsam::Pose2 current_pose =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  state(0) = current_pose.x();
  state(1) = current_pose.y();
  state(2) = current_pose.theta();
  unsigned int landmark_id = 3;
  for (auto it = _graph_values_.begin(); it != _graph_values_.end(); ++it) {
    // Iterate through the landmarks in the _graph_values_
    if (gtsam::Symbol(it->key).chr() == 'l') {  // If the key is a landmark (l)
      gtsam::Point2 landmark = it->value.cast<gtsam::Point2>();
      state(landmark_id++) = landmark.x();
      state(landmark_id++) = landmark.y();
    }
  }
  return state;
}

unsigned int GraphSLAMInstance::get_landmark_counter() const { return this->_landmark_counter_; }

unsigned int GraphSLAMInstance::get_pose_counter() const { return this->_pose_counter_; }

Eigen::MatrixXd GraphSLAMInstance::get_covariance_matrix() const {
  // if (this->_covariance_up_to_date_) {
  //   return _covariance_;
  // }
  // const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_);

  // // Create a gtsam::KeyVector with keys of all landmarks and the current pose, current pose
  // // first, landmarks ordered by id
  // gtsam::KeyVector keys;
  // keys.push_back(gtsam::Symbol('x', this->_pose_counter_));
  // for (unsigned int i = 1; i <= this->_landmark_counter_; i++) {
  //   keys.push_back(gtsam::Symbol('l', i));
  // }

  // this->_covariance_ = marginals.jointMarginalCovariance(keys).fullMatrix();
  // this->_covariance_up_to_date_ = true;

  // return _covariance_;
  return Eigen::MatrixXd::Zero(2 * this->_landmark_counter_, 2 * this->_landmark_counter_);
}

GraphSLAMInstance::GraphSLAMInstance(const SLAMParameters& params,
                                     std::shared_ptr<BaseOptimizer> optimizer)
    : _params_(params), _optimizer_(optimizer), _pose_timestamps_(params.max_pose_history_graph) {
  // Create a new factor graph
  _factor_graph_ = gtsam::NonlinearFactorGraph();
  const gtsam::Pose2 prior_pose(0.0, 0.0,
                                0.0);  // Create initial prior pose at origin, to bind graph
  const gtsam::Symbol pose_symbol('x', ++(this->_pose_counter_));
  const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.0, 0.0, 0.0));
  _factor_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(pose_symbol, prior_pose, prior_noise));
  if (const auto optimizer_ptr = std::dynamic_pointer_cast<ISAM2Optimizer>(this->_optimizer_)) {
    optimizer_ptr->_new_factors_.add(
        gtsam::PriorFactor<gtsam::Pose2>(pose_symbol, prior_pose, prior_noise));
    optimizer_ptr->_new_values_.insert(pose_symbol, prior_pose);
  }

  // Create a new values object
  _graph_values_ = gtsam::Values();
  _graph_values_.insert(pose_symbol, prior_pose);
  _new_pose_node_ = true;
}

GraphSLAMInstance::GraphSLAMInstance(const GraphSLAMInstance& other) {
  // Copy constructor
  _factor_graph_ = other._factor_graph_;
  _graph_values_ = other._graph_values_;
  _pose_counter_ = other._pose_counter_;
  _landmark_counter_ = other._landmark_counter_;
  _new_pose_node_ = other._new_pose_node_;
  _new_observation_factors_ = other._new_observation_factors_;
  _optimizer_ = other._optimizer_->clone();  // because it's a shared_ptr
  _params_ = other._params_;
  _pose_timestamps_ = other._pose_timestamps_;
}

GraphSLAMInstance& GraphSLAMInstance::operator=(const GraphSLAMInstance& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  _factor_graph_ = other._factor_graph_;
  _graph_values_ = other._graph_values_;
  _pose_counter_ = other._pose_counter_;
  _landmark_counter_ = other._landmark_counter_;
  _new_pose_node_ = other._new_pose_node_;
  _new_observation_factors_ = other._new_observation_factors_;
  _optimizer_ = other._optimizer_->clone();  // because it's a shared_ptr
  _params_ = other._params_;
  _pose_timestamps_ = other._pose_timestamps_;

  return *this;
}

void GraphSLAMInstance::process_new_pose(const Eigen::Vector3d& pose_difference,
                                         const Eigen::Vector3d& noise_vector,
                                         const Eigen::Vector3d& new_pose,
                                         const rclcpp::Time& pose_timestamp) {
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "GraphSLAMInstance - Processing new pose %lf %lf %lf",
               new_pose(0), new_pose(1), new_pose(2));
  gtsam::Pose2 new_pose_gtsam = eigen_to_gtsam_pose(new_pose);

  // X means pose node, l means landmark node
  gtsam::Symbol new_pose_symbol('x', ++(this->_pose_counter_));

  // Store the timestamp of the new pose in the circular buffer
  this->_pose_timestamps_.push(TimedPose{this->_pose_counter_, pose_timestamp});

  // Add the prior pose to the values
  _graph_values_.insert(new_pose_symbol, new_pose_gtsam);
  this->_new_pose_node_ = true;

  if (const auto optimizer_ptr = std::dynamic_pointer_cast<ISAM2Optimizer>(this->_optimizer_)) {
    optimizer_ptr->_new_values_.insert(new_pose_symbol, new_pose_gtsam);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("slam"),
               "GraphSLAMInstance - Adding prior pose %lf %lf %lf with noise %lf %lf %lf and "
               "pose difference %lf %lf %lf",
               new_pose_gtsam.x(), new_pose_gtsam.y(), new_pose_gtsam.theta(), noise_vector(0),
               noise_vector(1), noise_vector(2), pose_difference(0), pose_difference(1),
               pose_difference(2));

  this->process_pose_difference(pose_difference, noise_vector);
}

void GraphSLAMInstance::process_pose_difference(const Eigen::Vector3d& pose_difference,
                                                const Eigen::Vector3d& noise_vector,
                                                unsigned int before_pose_id,
                                                unsigned int after_pose_id) {
  gtsam::Symbol before_pose_symbol('x', before_pose_id);
  gtsam::Symbol after_pose_symbol('x', after_pose_id);
  gtsam::noiseModel::Diagonal::shared_ptr motion_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(noise_vector(0), noise_vector(1), noise_vector(2)));
  gtsam::Pose2 pose_difference_gtsam = eigen_to_gtsam_pose(pose_difference);
  this->_factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(before_pose_symbol, after_pose_symbol,
                                                              pose_difference_gtsam, motion_noise));
  if (const auto optimizer_ptr = std::dynamic_pointer_cast<ISAM2Optimizer>(this->_optimizer_)) {
    optimizer_ptr->_new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(
        before_pose_symbol, after_pose_symbol, pose_difference_gtsam, motion_noise));
  }
}

void GraphSLAMInstance::process_pose_difference(const Eigen::Vector3d& pose_difference,
                                                const Eigen::Vector3d& noise_vector,
                                                unsigned int before_pose_id) {
  this->process_pose_difference(pose_difference, noise_vector, before_pose_id,
                                this->_pose_counter_);
}

void GraphSLAMInstance::process_pose_difference(const Eigen::Vector3d& pose_difference,
                                                const Eigen::Vector3d& noise_vector) {
  this->process_pose_difference(pose_difference, noise_vector, this->_pose_counter_ - 1,
                                this->_pose_counter_);
}

void GraphSLAMInstance::process_observations(const ObservationData& observation_data) {
  Eigen::VectorXd& observations = *(observation_data.observations_);
  Eigen::VectorXi& associations = *(observation_data.associations_);
  Eigen::VectorXd& observations_global = *(observation_data.observations_global_);
  bool new_observation_factors = false;
  for (unsigned int i = 0; i < observations.size() / 2; i++) {
    gtsam::Point2 landmark;
    gtsam::Symbol landmark_symbol;
    bool landmark_lost_in_optimization =
        static_cast<int>(this->_landmark_counter_) < (associations(i)) / 2;
    if (associations(i) == -2) {
      // No association
      continue;
    } else if (associations(i) == -1 || landmark_lost_in_optimization) {
      // Create new landmark
      landmark_symbol = gtsam::Symbol('l', ++(this->_landmark_counter_));
      landmark = gtsam::Point2(observations_global(i * 2), observations_global(i * 2 + 1));
      this->_graph_values_.insert(landmark_symbol, landmark);
      if (const auto optimizer_ptr = std::dynamic_pointer_cast<ISAM2Optimizer>(this->_optimizer_)) {
        optimizer_ptr->_new_values_.insert(
            landmark_symbol, landmark);  // Most efficient way I found to deal with ISAM2
      }
    } else {
      if (!this->new_pose_factors()) {  // Only add old observations if the vehicle has moved
        continue;
      }
      // Association to previous landmark
      landmark_symbol = gtsam::Symbol('l', (associations(i)) / 2 + 1);  // Convert to landmark id
    }
    new_observation_factors = true;
    const Eigen::Vector2d observation_cartesian(observations[i * 2], observations[i * 2 + 1]);
    Eigen::Vector2d observation_cylindrical = common_lib::maths::cartesian_to_cylindrical(
        observation_cartesian);  // Convert to cylindrical coordinates
    const gtsam::Rot2 observation_rotation(observation_cylindrical(1));

    const gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(this->_params_.observation_y_noise_,
                                                           this->_params_.observation_x_noise_));

    if (const auto optimizer_ptr = std::dynamic_pointer_cast<ISAM2Optimizer>(this->_optimizer_)) {
      optimizer_ptr->_new_factors_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
          gtsam::Symbol('x', this->get_landmark_id_at_time(observation_data.timestamp_)),
          landmark_symbol, observation_rotation, observation_cylindrical(0),
          observation_noise));  // Again, just for ISAM2, because
                                // it's incremental
    }
    this->_factor_graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
        gtsam::Symbol('x', this->get_landmark_id_at_time(observation_data.timestamp_)),
        landmark_symbol, observation_rotation, observation_cylindrical(0), observation_noise));
  }
  this->_new_pose_node_ = !new_observation_factors;
  this->_new_observation_factors_ = new_observation_factors;
}

void GraphSLAMInstance::load_initial_state(const Eigen::VectorXd& map, const Eigen::Vector3d& pose,
                                           double preloaded_map_noise) {
  RCLCPP_INFO(rclcpp::get_logger("slam"), "GraphSLAMInstance - Loading map ");
  this->_pose_counter_ = 0;
  this->_landmark_counter_ = 0;
  _factor_graph_ = gtsam::NonlinearFactorGraph();
  const gtsam::Pose2 prior_pose(pose(0), pose(1), pose(2));
  const gtsam::Symbol pose_symbol('x', ++(this->_pose_counter_));
  const gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.0, 0.0, 0.0));
  _factor_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(pose_symbol, prior_pose, prior_noise));

  _graph_values_ = gtsam::Values();
  _graph_values_.insert(pose_symbol, prior_pose);
  _new_pose_node_ = true;

  unsigned int num_landmarks = map.size() / 2;
  for (unsigned int i = 0; i < num_landmarks; i++) {
    gtsam::Point2 landmark(map(i * 2), map(i * 2 + 1));
    gtsam::Symbol landmark_symbol('l', ++(this->_landmark_counter_));
    _graph_values_.insert(landmark_symbol, landmark);
    const gtsam::noiseModel::Diagonal::shared_ptr landmark_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector2(preloaded_map_noise, preloaded_map_noise));
    _factor_graph_.add(
        gtsam::PriorFactor<gtsam::Point2>(landmark_symbol, landmark, landmark_noise));
  }
}

void GraphSLAMInstance::optimize() {
  RCLCPP_DEBUG(rclcpp::get_logger("slam"),
               "GraphSLAMInstance - Optimizing graph with %ld factors and %ld values",
               this->_factor_graph_.size(), this->_graph_values_.size());
  if (!this->_new_observation_factors_) return;

  this->_graph_values_ = this->_optimizer_->optimize(
      this->_factor_graph_, this->_graph_values_, this->_pose_counter_, this->_landmark_counter_);
  this->_new_observation_factors_ = false;
}

void GraphSLAMInstance::lock_landmarks(double locked_landmark_noise) {
  // TODO: check if this works
  for (unsigned int i = 1; i < this->_landmark_counter_; i++) {
    gtsam::Symbol landmark_symbol('l', i);
    if (this->_graph_values_.exists(landmark_symbol)) {
      gtsam::Point2 landmark = this->_graph_values_.at<gtsam::Point2>(landmark_symbol);
      const gtsam::noiseModel::Diagonal::shared_ptr landmark_noise =
          gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector2(locked_landmark_noise, locked_landmark_noise));
      this->_factor_graph_.add(
          gtsam::PriorFactor<gtsam::Point2>(landmark_symbol, landmark, landmark_noise));
    }
  }
}

unsigned int GraphSLAMInstance::get_landmark_id_at_time(const rclcpp::Time& timestamp) const {
  if (_pose_timestamps_.size() == 0) {
    throw std::runtime_error("No poses in buffer");
  }

  for (size_t i = 0; i < _pose_timestamps_.size(); ++i) {
    const TimedPose& curr = _pose_timestamps_.from_end(i);

    if (curr.timestamp <= timestamp) {
      if (i == 0) return curr.pose_id;  // AVoid out-of-bounds
      const TimedPose& older = _pose_timestamps_.from_end(i - 1);

      auto diff_curr = timestamp.nanoseconds() - curr.timestamp.nanoseconds();
      auto diff_older = older.timestamp.nanoseconds() - timestamp.nanoseconds();
      return (diff_curr <= diff_older) ? curr.pose_id : older.pose_id;
    }
  }

  // If no pose <= timestamp, return oldest pose
  return _pose_timestamps_.from_end(_pose_timestamps_.size() - 1).pose_id;
}
