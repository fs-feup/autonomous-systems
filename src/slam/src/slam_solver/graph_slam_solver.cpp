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

#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/transformations.hpp"
#include <perception_sensor_lib/loop_closure/loop_closure.hpp>

void GraphSLAMSolver::_init() {
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

GraphSLAMSolver::GraphSLAMSolver(const SLAMParameters& params,
                                 std::shared_ptr<DataAssociationModel> data_association,
                                 std::shared_ptr<V2PMotionModel> motion_model,
                                 std::shared_ptr<std::vector<double>> execution_times)
    : SLAMSolver(params, data_association, motion_model, execution_times) {
  _init();
}

GraphSLAMSolver::GraphSLAMSolver(const SLAMParameters& params,
                                 std::shared_ptr<DataAssociationModel> data_association,
                                 std::shared_ptr<V2PMotionModel> motion_model,
                                 std::shared_ptr<std::vector<double>> execution_times,
                                 std::weak_ptr<rclcpp::Node> node)
    : SLAMSolver(params, data_association, motion_model, execution_times, node) {
  _init();

  // Create a timer for asynchronous optimization
  if (params.slam_optimization_period_ <= 0.0) {
    return;
  } else if (const auto node_ptr = _node_.lock()) {
    this->_optimization_timer_ = node_ptr->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(params.slam_optimization_period_)),
        [this]() { this->_optimize(); });
  } else {
    this->_params_.slam_optimization_period_ = 0.0;  // Trigger optimization on observations
    RCLCPP_ERROR(rclcpp::get_logger("slam"),
                 "Failed to create optimization timer, node is not valid");
  }
}

void GraphSLAMSolver::add_motion_prior(const common_lib::structures::Velocities& velocities) {
  if (!this->_received_first_velocities_) {
    _last_pose_update_ = velocities.timestamp_;
    this->_received_first_velocities_ = true;
    return;
  }

  // Apply Motion Model
  const double delta_t =
      (velocities.timestamp_ - this->_last_pose_update_).seconds();  // Get the time

  const Eigen::Vector3d velocities_vector(velocities.velocity_x, velocities.velocity_y,
                                          velocities.rotational_velocity);
  const Eigen::Vector3d previous_pose(this->_last_pose_.x(), this->_last_pose_.y(),
                                      this->_last_pose_.theta());
  const Eigen::Vector3d next_pose =
      this->_motion_model_->get_next_pose(previous_pose, velocities_vector, delta_t);
  this->_last_pose_ = gtsam::Pose2(next_pose[0], next_pose[1], next_pose[2]);

  // Calculate pose difference
  const gtsam::Pose2 previous_graphed_pose =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  gtsam::Pose2 pose_difference(this->_last_pose_.x() - previous_graphed_pose.x(),
                               this->_last_pose_.y() - previous_graphed_pose.y(),
                               this->_last_pose_.theta() - previous_graphed_pose.theta());

  // Update the last pose update
  _last_pose_update_ = velocities.timestamp_;

  // If the difference is too small, do not add the factor
  if (const double pose_difference_norm =
          sqrt(pow(pose_difference.x(), 2) + pow(pose_difference.y(), 2) +
               pow(pose_difference.theta(), 2));
      pose_difference_norm < this->_params_.slam_min_pose_difference_) {
    return;
  }

  // Calculate noise
  // // TODO: Implement noise -> this was shit because covariance from velocity estimation had too
  // // small error
  // Eigen::Matrix3d velocities_noise = Eigen::Matrix3d::Identity(); velocities_noise(0,
  // 0) = velocities.velocity_x_noise_; velocities_noise(1, 1) = velocities.velocity_y_noise_;
  // velocities_noise(2, 2) = velocities.rotational_velocity_noise_;
  // RCLCPP_DEBUG(rclcpp::get_logger("slam"), "Velocities noise: %f %f %f", velocities_noise(0, 0),
  //              velocities_noise(1, 1), velocities_noise(2, 2));
  // Eigen::Matrix3d velocities_jacobian =
  //     this->_motion_model_->get_jacobian_velocities(previous_pose, velocities_vector, delta_t);
  // Eigen::Matrix3d motion_covariance =
  //     velocities_jacobian * velocities_noise * velocities_jacobian.transpose();
  // RCLCPP_DEBUG(rclcpp::get_logger("slam"), "Motion covariance: %f %f %f", motion_covariance(0,
  // 0),
  //              motion_covariance(1, 1), motion_covariance(2, 2));
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));

  // Add the prior factor to the graph
  gtsam::Symbol previous_pose_symbol('x', this->_pose_counter_);
  gtsam::Symbol new_pose_symbol('x', ++(this->_pose_counter_));

  _factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previous_pose_symbol, new_pose_symbol,
                                                        pose_difference, prior_noise));
  _covariance_up_to_date_ = false;
  this->_updated_pose_ = true;

  // Add the prior pose to the values
  _graph_values_.insert(new_pose_symbol, this->_last_pose_);
}

void GraphSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
  // Only proceed if the vehicle has moved
  if (!this->_updated_pose_ && !this->_first_observation_) {
    return;
  }

  this->_first_observation_ = false;

  rclcpp::Time start_time = rclcpp::Clock().now();

  // Prepare structures for data association
  Eigen::VectorXd observations(cones.size() * 2);
  Eigen::VectorXd observations_confidences(cones.size());
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

  rclcpp::Time initialization_time = rclcpp::Clock().now();

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(
      observations.size(), observations.size());  // TODO(marhcouto): true covariance

  rclcpp::Time covariance_time = rclcpp::Clock().now();

  // Data association
  Eigen::VectorXi associations(cones.size());
  associations = this->_data_association_->associate(
      state, covariance, observations,
      observations_confidences);  // TODO: implement different mahalanobis distance

  LoopClosure loop_closure(5, 6);
  std::pair<bool, double> result = loop_closure.detect(current_pose, cones, associations, cones);

  rclcpp::Time association_time = rclcpp::Clock().now();

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
      landmark_symbol =
          gtsam::Symbol('l', (associations(i) - 3) / 2 + 1);  // Convert to landmark id
    }
    const Eigen::Vector2d observation_cartesian(cones[i].position.x, cones[i].position.y);
    Eigen::Vector2d observation_cylindrical = common_lib::maths::cartesian_to_cylindrical(
        observation_cartesian);  // Convert to cylindrical coordinates
    const gtsam::Rot2 observation_rotation(observation_cylindrical(1));

    const gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(this->_params_.observation_x_noise_,
                                                           this->_params_.observation_y_noise_));

    this->_factor_graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
        gtsam::Symbol('x', this->_pose_counter_), landmark_symbol, observation_rotation,
        observation_cylindrical(0), observation_noise));
    _covariance_up_to_date_ = false;
  }
  this->_updated_pose_ = false;

  rclcpp::Time factor_graph_time = rclcpp::Clock().now();

  // Optimize the graph
  if (this->_params_.slam_optimization_period_ == 0.0) {  // If optimization is synchronous
    this->_optimize();
  }

  // Timekeeping
  rclcpp::Time final_time = rclcpp::Clock().now();
  if (this->_execution_times_ == nullptr) {
    return;
  }
  this->_execution_times_->at(3) = (covariance_time - initialization_time).seconds() * 1000.0;
  this->_execution_times_->at(2) = (association_time - covariance_time).seconds() * 1000.0;
  this->_execution_times_->at(4) = (factor_graph_time - association_time).seconds() * 1000.0;
}

void GraphSLAMSolver::_optimize() {  // TODO: mutexes and parallelization and implement sliding
                                     // window mode
  rclcpp::Time start_time = rclcpp::Clock().now();
  gtsam::LevenbergMarquardtOptimizer optimizer(this->_factor_graph_, this->_graph_values_);
  this->_graph_values_ = optimizer.optimize();

  rclcpp::Time optimization_time = rclcpp::Clock().now();
  this->_last_pose_ = this->_graph_values_.at<gtsam::Pose2>(
      gtsam::Symbol('x', this->_pose_counter_));  // Update last pose
  if (this->_execution_times_ == nullptr) {
    return;
  }

  this->_execution_times_->at(5) = (optimization_time - start_time).seconds() * 1000.0;
}

Eigen::MatrixXd GraphSLAMSolver::get_covariance() {
  if (this->_covariance_up_to_date_) {
    return _covariance_;
  }
  const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_);

  // Create a gtsam::KeyVector with keys of all landmarks and the current pose, current pose
  // first, landmarks ordered by id
  gtsam::KeyVector keys;
  keys.push_back(gtsam::Symbol('x', this->_pose_counter_));
  for (unsigned int i = 1; i <= this->_landmark_counter_; i++) {
    keys.push_back(gtsam::Symbol('l', i));
  }

  this->_covariance_ = marginals.jointMarginalCovariance(keys).fullMatrix();
  this->_covariance_up_to_date_ = true;

  return _covariance_;
}

std::vector<common_lib::structures::Cone>
GraphSLAMSolver::get_map_estimate() {  // TODO: include the map updates in observation to save time
  rclcpp::Time current_time = rclcpp::Clock().now();
  std::vector<common_lib::structures::Cone> map_estimate;
  // const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_);
  for (auto it = _graph_values_.begin(); it != _graph_values_.end(); ++it) {
    // Iterate through the landmarks in the _graph_values_
    if (gtsam::Symbol(it->key).chr() == 'l') {
      // Get landmarks covariance
      // const gtsam::Matrix covariance = marginals.marginalCovariance(it->key); // TODO:
      // covariances are slow af
      gtsam::Point2 landmark = it->value.cast<gtsam::Point2>();
      common_lib::structures::Position landmark_position(landmark.x(), landmark.y(), 0.0, 0.0,
                                                         current_time);
      map_estimate.push_back(common_lib::structures::Cone(
          landmark_position, common_lib::competition_logic::Color::UNKNOWN, 1.0,
          current_time));  // TODO: vary cone confidence
    }
  }
  return map_estimate;
}

common_lib::structures::Pose GraphSLAMSolver::get_pose_estimate() {
  // const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_);
  // const gtsam::Key key = gtsam::Symbol('x', this->_pose_counter_);
  // const gtsam::Matrix covariance = marginals.marginalCovariance(key);
  return common_lib::structures::Pose(_last_pose_.x(), _last_pose_.y(), _last_pose_.theta(), 0.0,
                                      0.0, 0.0, _last_pose_update_);
}
