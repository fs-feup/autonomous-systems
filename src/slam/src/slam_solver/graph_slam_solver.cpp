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

gtsam::Pose2 eigen_to_gtsam_pose(const Eigen::Vector3d& pose) {
  return gtsam::Pose2(pose[0], pose[1], pose[2]);
}

Eigen::Vector3d gtsam_pose_to_eigen(const gtsam::Pose2& pose) {
  return Eigen::Vector3d(pose.x(), pose.y(), pose.theta());
}

bool GraphSLAMInstance::should_process_observations() const { return this->_new_pose_node_; }

bool GraphSLAMInstance::should_perform_optimization() const {
  return this->_new_observation_factors_;
}

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
    if (gtsam::Symbol(it->key).chr() == 'l') {
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
  return Eigen::MatrixXd::Zero(this->_landmark_counter_, this->_landmark_counter_);
}

GraphSLAMInstance::GraphSLAMInstance(const SLAMParameters& params) : _params_(params) {
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

  return *this;
}

void GraphSLAMInstance::process_pose(const gtsam::Pose2& pose) {
  const gtsam::Pose2 last_pose_node =
      this->_graph_values_.at<gtsam::Pose2>(gtsam::Symbol('x', this->_pose_counter_));
  const gtsam::Pose2 pose_difference = pose.between(last_pose_node);

  if (const double pose_difference_norm =
          ::sqrt(pow(pose_difference.x(), 2) + pow(pose_difference.y(), 2) +
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

  this->_factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previous_pose_symbol, new_pose_symbol,
                                                              pose_difference, prior_noise));
  this->_new_pose_node_ = true;

  // Add the prior pose to the values
  _graph_values_.insert(new_pose_symbol, pose);
}

void GraphSLAMInstance::process_observations(const ObservationData& observation_data) {
  Eigen::VectorXd& observations = *(observation_data.observations_);
  Eigen::VectorXi& associations = *(observation_data.associations_);
  Eigen::VectorXd& observations_global = *(observation_data.observations_global_);
  for (unsigned int i = 0; i < observations.size() / 2; i++) {
    gtsam::Point2 landmark;
    gtsam::Symbol landmark_symbol;
    bool landmark_lost_in_optimization =
        static_cast<int>(this->_landmark_counter_) < (associations(i) - 3) / 2 + 1;
    if (associations(i) == -2) {
      // No association
      continue;
    } else if (associations(i) == -1 || landmark_lost_in_optimization) {
      // Create new landmark
      landmark_symbol = gtsam::Symbol('l', ++(this->_landmark_counter_));
      landmark = gtsam::Point2(observations_global(i * 2), observations_global(i * 2 + 1));
      this->_graph_values_.insert(landmark_symbol, landmark);
    } else {
      // Association to previous landmark
      landmark_symbol =
          gtsam::Symbol('l', (associations(i) - 3) / 2 + 1);  // Convert to landmark id
    }
    const Eigen::Vector2d observation_cartesian(observations[i * 2], observations[i * 2 + 1]);
    Eigen::Vector2d observation_cylindrical = common_lib::maths::cartesian_to_cylindrical(
        observation_cartesian);  // Convert to cylindrical coordinates
    const gtsam::Rot2 observation_rotation(observation_cylindrical(1));

    const gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(this->_params_.observation_x_noise_,
                                                           this->_params_.observation_y_noise_));

    this->_factor_graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
        gtsam::Symbol('x', this->_pose_counter_), landmark_symbol, observation_rotation,
        observation_cylindrical(0), observation_noise));
  }
  this->_new_pose_node_ = false;
  this->_new_observation_factors_ = true;
}

void GraphSLAMInstance::optimize() {  // TODO: implement sliding window and other parameters
  gtsam::LevenbergMarquardtOptimizer optimizer(this->_factor_graph_, this->_graph_values_);
  this->_graph_values_ = optimizer.optimize();
  this->_new_observation_factors_ = false;
}

GraphSLAMSolver::GraphSLAMSolver(const SLAMParameters& params,
                                 std::shared_ptr<DataAssociationModel> data_association,
                                 std::shared_ptr<V2PMotionModel> motion_model,
                                 std::shared_ptr<std::vector<double>> execution_times)
    : SLAMSolver(params, data_association, motion_model, execution_times),
      _graph_slam_instance_(params) {
  // TODO: transform into range and bearing noises
}

// ------------------------------ PoseUpdater --------------------------------------

void PoseUpdater::update_pose(const MotionData& motion_data,
                              std::shared_ptr<V2PMotionModel> motion_model) {
  if (!this->_received_first_velocities_) {
    this->_last_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->_last_pose_update_ = motion_data.timestamp_;
    this->_received_first_velocities_ = true;
    return;
  }

  this->_last_pose_ =
      motion_model->get_next_pose(this->_last_pose_, *(motion_data.velocities_),
                                  (motion_data.timestamp_ - this->_last_pose_update_).seconds());
  this->_last_pose_update_ = motion_data.timestamp_;
}

PoseUpdater::PoseUpdater(const PoseUpdater& other) {
  this->_last_pose_ = other._last_pose_;
  this->_last_pose_update_ = other._last_pose_update_;
  this->_received_first_velocities_ = other._received_first_velocities_;
}

PoseUpdater& PoseUpdater::operator=(const PoseUpdater& other) {
  if (this == &other) return *this;  // Prevent self-assignment

  // Copy each member individually
  this->_last_pose_ = other._last_pose_;
  this->_last_pose_update_ = other._last_pose_update_;
  this->_received_first_velocities_ = other._received_first_velocities_;

  return *this;
}

// ------------------------------- GraphSLAMSolver ---------------------------------

void GraphSLAMSolver::init(std::weak_ptr<rclcpp::Node> node) {
  // Create a timer for asynchronous optimization
  if (this->_params_.slam_optimization_period_ <= 0.0) {
    RCLCPP_INFO(rclcpp::get_logger("slam"),
                "Optimization period is zero, optimization is synchronous");
  } else if (const auto node_ptr = node.lock()) {
    this->_reentrant_group_ = node_ptr->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);  // Allow callbacks to execute in parallel
    this->_optimization_timer_ = node_ptr->create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(this->_params_.slam_optimization_period_ * 1000)),
        [this]() { this->_asynchronous_optimization_routine(); }, this->_reentrant_group_);
    RCLCPP_INFO(rclcpp::get_logger("slam"), "Optimization timer created with period %f seconds",
                this->_params_.slam_optimization_period_);
  } else {
    this->_params_.slam_optimization_period_ = 0.0;  // Trigger optimization on observations
    RCLCPP_ERROR(rclcpp::get_logger("slam"),
                 "Failed to create optimization timer, node is not valid");
  }
}

void GraphSLAMSolver::add_motion_prior(const common_lib::structures::Velocities& velocities) {
  std::unique_lock lock(this->_mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_motion_prior - Mutex accessed");
  MotionData velocities_data(
      std::make_shared<Eigen::Vector3d>(velocities.velocity_x, velocities.velocity_y,
                                        velocities.rotational_velocity),
      velocities.timestamp_);
  this->_pose_updater_.update_pose(velocities_data, this->_motion_model_);
  if (this->_optimization_under_way_) {
    this->_motion_data_queue_.push(velocities_data);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_motion_prior - Optimization under way, pushing motion data");
  }
  this->_graph_slam_instance_.process_pose(
      eigen_to_gtsam_pose(this->_pose_updater_.get_last_pose()));
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_motion_prior - Mutex unlocked");
}

void GraphSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
  rclcpp::Time start_time, initialization_time, covariance_time, association_time,
      factor_graph_time, optimization_time;
  start_time = rclcpp::Clock().now();
  Eigen::VectorXd observations(cones.size() * 2);
  Eigen::VectorXd observations_confidences(cones.size());
  Eigen::VectorXd observations_global(cones.size() * 2);
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
  for (unsigned int i = 0; i < cones.size(); i++) {
    // Iterate through the cones
    observations(i * 2) = cones[i].position.x;
    observations(i * 2 + 1) = cones[i].position.y;
    observations_confidences(i) = cones[i].certainty;
  }

  {
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Shared mutex accessed");
    const std::shared_lock lock(this->_mutex_);
    // Only proceed if the vehicle has moved
    if (!this->_graph_slam_instance_.should_process_observations()) {
      return;
    }
    state = this->_graph_slam_instance_.get_state_vector();
    observations_global =
        common_lib::maths::local_to_global_coordinates(state.head<3>(), observations);
    initialization_time = rclcpp::Clock().now();
    covariance = this->_graph_slam_instance_.get_covariance_matrix();
    covariance_time = rclcpp::Clock().now();
  }

  // Data association
  Eigen::VectorXi associations(cones.size());
  associations = this->_data_association_->associate(
      state, covariance, observations,
      observations_confidences);  // TODO: implement different mahalanobis distance
  association_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Associations calculated");

  {
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_observations - Mutex locked - processing observations");
    std::unique_lock uniq_lock(this->_mutex_);
    ObservationData observation_data(std::make_shared<Eigen::VectorXd>(observations),
                                     std::make_shared<Eigen::VectorXi>(associations),
                                     std::make_shared<Eigen::VectorXd>(observations_global),
                                     cones.at(0).timestamp);
    if (this->_optimization_under_way_) {
      this->_observation_data_queue_.push(observation_data);
      RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                   "add_observations - Optimization under way, pushing observation data");
    }
    this->_graph_slam_instance_.process_observations(observation_data);
  }
  factor_graph_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Mutex unlocked - Factors added");

  // Optimize the graph
  if (this->_params_.slam_optimization_period_ <= 0.0 &&
      this->_graph_slam_instance_
          .should_perform_optimization()) {  // If optimization is synchronous
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Mutex locked - optimizing graph");
    std::unique_lock uniq_lock(this->_mutex_);
    this->_graph_slam_instance_.optimize();
    optimization_time = rclcpp::Clock().now();
    this->_pose_updater_.set_last_pose(gtsam_pose_to_eigen(this->_graph_slam_instance_.get_pose()));
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Mutex unlocked - graph optimized");
  }

  // Timekeeping
  if (this->_execution_times_ == nullptr) {
    return;
  }
  this->_execution_times_->at(3) = (covariance_time - initialization_time).seconds() * 1000.0;
  this->_execution_times_->at(2) = (association_time - covariance_time).seconds() * 1000.0;
  this->_execution_times_->at(4) = (factor_graph_time - association_time).seconds() * 1000.0;
  if (this->_params_.slam_optimization_period_ <= 0.0) {
    this->_execution_times_->at(5) = (optimization_time - factor_graph_time).seconds() * 1000.0;
  }
}

void GraphSLAMSolver::_asynchronous_optimization_routine() {
  rclcpp::Time start_time = rclcpp::Clock().now();
  GraphSLAMInstance graph_slam_instance_copy;
  PoseUpdater pose_updater_copy;
  {
    const std::shared_lock lock(this->_mutex_);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "_asynchronous_optimization_routine - Shared mutex accessed");
    if (!this->_graph_slam_instance_.should_perform_optimization()) {
      return;
    }
    graph_slam_instance_copy = this->_graph_slam_instance_;
    pose_updater_copy = this->_pose_updater_;
  }
  {
    std::unique_lock lock(this->_mutex_);
    this->_optimization_under_way_ = true;
  }
  rclcpp::Time initialization_time = rclcpp::Clock().now();
  // Optimize the graph
  RCLCPP_DEBUG(rclcpp::get_logger("slam"),
               "_asynchronous_optimization_routine - Starting optimization");
  graph_slam_instance_copy.optimize();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "_asynchronous_optimization_routine - Graph optimized");
  rclcpp::Time optimization_time = rclcpp::Clock().now();
  {
    std::unique_lock lock(this->_mutex_);
    this->_optimization_under_way_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "_asynchronous_optimization_routine - Mutex locked");
    while (this->_motion_data_queue_.size() > 0 || this->_observation_data_queue_.size() > 0) {
      bool process_pose = this->_observation_data_queue_.size() <= 0 ||
                          (this->_motion_data_queue_.size() > 0 &&
                           this->_motion_data_queue_.front().timestamp_ <
                               this->_observation_data_queue_.front().timestamp_);
      if (process_pose) {
        pose_updater_copy.update_pose(this->_motion_data_queue_.front(), this->_motion_model_);
        graph_slam_instance_copy.process_pose(
            eigen_to_gtsam_pose(pose_updater_copy.get_last_pose()));
        this->_motion_data_queue_.pop();
      } else {
        graph_slam_instance_copy.process_observations(this->_observation_data_queue_.front());
        this->_observation_data_queue_.pop();
      }
      // Print number of landmarks and poses
      RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                   "_asynchronous_optimization_routine - Number of landmarks: %d",
                   graph_slam_instance_copy.get_landmark_counter());
      RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                   "_asynchronous_optimization_routine - Number of poses: %d",
                   graph_slam_instance_copy.get_pose_counter());
    }
    this->_graph_slam_instance_ = graph_slam_instance_copy;
    this->_pose_updater_ = pose_updater_copy;
  }
  rclcpp::Time end_time = rclcpp::Clock().now();
  if (this->_execution_times_ != nullptr) {
    this->_execution_times_->at(5) =
        (optimization_time - initialization_time).seconds() * 1000.0;  // Optimization time
    this->_execution_times_->at(6) =
        (initialization_time - start_time).seconds() * 1000.0;  // Initialization time
    this->_execution_times_->at(7) =
        (end_time - optimization_time).seconds() * 1000.0;  // Graph update time
    this->_execution_times_->at(8) =
        (end_time - start_time).seconds() * 1000.0;  // Optimization time
  }
  RCLCPP_DEBUG(rclcpp::get_logger("slam"),
               "_asynchronous_optimization_routine - Mutex unlocked - Graph updated");
}

std::vector<common_lib::structures::Cone>
GraphSLAMSolver::get_map_estimate() {  // TODO: include the map updates in observation to save time
  rclcpp::Time current_time = rclcpp::Clock().now();
  const gtsam::Values& graph_values = this->_graph_slam_instance_.get_graph_values_reference();
  std::vector<common_lib::structures::Cone> map_estimate;
  map_estimate.reserve(this->_graph_slam_instance_.get_landmark_counter());
  // const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_);
  for (auto it = graph_values.begin(); it != graph_values.end(); ++it) {
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

  Eigen::Vector3d pose_vector = this->_pose_updater_.get_last_pose();

  return common_lib::structures::Pose(pose_vector(0), pose_vector(1), pose_vector(2), 0.0, 0.0, 0.0,
                                      this->_pose_updater_.get_last_pose_update());
}

Eigen::MatrixXd GraphSLAMSolver::get_covariance() {
  return this->_graph_slam_instance_.get_covariance_matrix();
}
