#include "slam_solver/graph_slam_solver/graph_slam_solver.hpp"

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

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/transformations.hpp"
#include "slam_solver/graph_slam_solver/optimizer/map.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/map.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/pose_updater_traits/second_pose_input_trait.hpp"
#include "slam_solver/graph_slam_solver/utils.hpp"

GraphSLAMSolver::GraphSLAMSolver(const SLAMParameters& params,
                                 std::shared_ptr<DataAssociationModel> data_association,
                                 std::shared_ptr<V2PMotionModel> motion_model,
                                 std::shared_ptr<LandmarkFilter> landmark_filter,
                                 std::shared_ptr<std::vector<double>> execution_times,
                                 std::shared_ptr<LoopClosure> loop_closure)
    : SLAMSolver(params, data_association, motion_model, landmark_filter, execution_times,
                 loop_closure) {
  this->_graph_slam_instance_ = std::make_shared<GraphSLAMInstance>(
      params, graph_slam_optimizer_constructors_map.at(params.slam_optimization_type_)(params));
  this->_pose_updater_ = pose_updater_constructors_map.at(params.pose_updater_name_)(params);
  this->_odometry_model = std::make_shared<OdometryModel>();
}

void GraphSLAMSolver::init([[maybe_unused]] std::weak_ptr<rclcpp::Node> node) {
  if (const auto node_ptr = node.lock()) {
    this->_reentrant_group_ = node_ptr->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);  // Allow callbacks to execute in parallel
  }
  // Create a timer for asynchronous optimization
  if (this->_params_.slam_optimization_mode_ != "async" ||
      this->_params_.slam_optimization_period_ <= 0.0) {
    RCLCPP_INFO(rclcpp::get_logger("slam"), "Optimization is synchronous");
  } else if (const auto node_ptr = node.lock()) {
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

bool GraphSLAMSolver::_add_motion_data_to_graph(
    const std::shared_ptr<PoseUpdater> pose_updater,
    const std::shared_ptr<GraphSLAMInstance> graph_slam_instance, bool force_update) {
  if (pose_updater->pose_ready_for_graph_update() || force_update) {
    // If the pose updater is ready to update the pose, we can proceed with the pose update
    // Eigen::Vector3d pose_difference_noise = {0.2, 0.2, 0.3};
    graph_slam_instance->process_new_pose(
        pose_difference_eigen(pose_updater->get_last_graphed_pose(), pose_updater->get_last_pose()),
        pose_updater->get_pose_difference_noise(), pose_updater->get_last_pose(),
        pose_updater->get_last_pose_update());

    pose_updater->update_pose(
        pose_updater->get_last_pose());  // Reset the accumulated pose difference
    return true;
  }

  return false;
}

void GraphSLAMSolver::add_odometry(const common_lib::structures::Pose& odometry) {
  std::unique_lock lock(this->_mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - Mutex accessed");
  rclcpp::Time start_time, motion_model_time, factor_graph_time;

  // Apply motion model
  MotionData odometry_data(std::make_shared<Eigen::VectorXd>(3),
                           std::make_shared<Eigen::VectorXd>(3), odometry.timestamp);
  odometry_data.motion_data_->head<3>() << odometry.position.x, odometry.position.y,
      odometry.orientation;
  odometry_data.motion_data_noise_->head<3>() << odometry.position.x_noise,
      odometry.position.y_noise, odometry.orientation_noise;
  start_time = rclcpp::Clock().now();
  if (this->_optimization_under_way_) {
    this->_motion_data_queue_.push(odometry_data);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_odometry - Optimization under way, pushing motion data");
  }
  this->_pose_updater_->predict_pose(odometry_data, this->_odometry_model);
  motion_model_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - Pose updated");

  // Add motion data to the graph
  bool added_factors =
      this->_add_motion_data_to_graph(this->_pose_updater_, this->_graph_slam_instance_);
  factor_graph_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - Factors added? %d", added_factors);

  // Timekeeping
  if (this->_execution_times_ == nullptr) {
    return;
  }
  this->_execution_times_->at(9) = (motion_model_time - start_time).seconds() * 1000.0;
  this->_execution_times_->at(10) = (factor_graph_time - motion_model_time).seconds() * 1000.0;
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - Mutex unlocked");
}

void GraphSLAMSolver::add_velocities(const common_lib::structures::Velocities& velocities) {
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Mutex waiting to lock");
  std::unique_lock lock(this->_mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Mutex accessed");
  rclcpp::Time start_time, motion_model_time, factor_graph_time;

  // Construct motion data
  MotionData velocities_data(std::make_shared<Eigen::VectorXd>(4),
                             std::make_shared<Eigen::VectorXd>(4), velocities.timestamp_);
  velocities_data.motion_data_->head<4>() << velocities.velocity_x, velocities.velocity_y,
      velocities.rotational_velocity, this->_last_imu_data_.acceleration_x;
  velocities_data.motion_data_noise_->head<4>() << _params_.velocity_x_noise_,
      _params_.velocity_y_noise_, _params_.angular_velocity_noise_,
      _params_.imu_acceleration_x_noise_;
  velocities_data.timestamp_ = velocities.timestamp_;
  start_time = rclcpp::Clock().now();
  if (this->_optimization_under_way_) {
    this->_motion_data_queue_.push(velocities_data);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_velocities - Optimization under way, pushing motion data");
  }

  // Apply motion model
  this->_pose_updater_->predict_pose(velocities_data, this->_motion_model_);
  motion_model_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Pose updated: (%f, %f, %f)",
               this->_pose_updater_->get_last_pose()(0), this->_pose_updater_->get_last_pose()(1),
               this->_pose_updater_->get_last_pose()(2));

  // Add motion data to the graph
  bool added_factors =
      this->_add_motion_data_to_graph(this->_pose_updater_, this->_graph_slam_instance_);
  factor_graph_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Factors added? %d", added_factors);

  // Timekeeping
  if (this->_execution_times_ == nullptr) {
    return;
  }
  this->_execution_times_->at(9) = (motion_model_time - start_time).seconds() * 1000.0;
  this->_execution_times_->at(10) = (factor_graph_time - motion_model_time).seconds() * 1000.0;
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Mutex unlocked");
}

void GraphSLAMSolver::add_imu_data(const common_lib::sensor_data::ImuData& imu_data) {
  this->_last_imu_data_ = imu_data;
}

void GraphSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones,
                                       rclcpp::Time cones_timestamp) {
  if (cones.empty()) {
    // this->_landmark_filter_->filter(Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0),
    //                                 Eigen::VectorXi::Zero(0));
    this->_observations_global_ = Eigen::VectorXd::Zero(0);
    this->_associations_ = Eigen::VectorXi::Zero(0);
    return;
  }

  // Prepare the data structures for observations
  rclcpp::Time start_time, initialization_time, covariance_time, association_time,
      factor_graph_time, optimization_time;
  start_time = rclcpp::Clock().now();
  Eigen::VectorXd observations(cones.size() * 2);
  Eigen::VectorXd observations_confidences(cones.size());
  Eigen::VectorXd observations_global(cones.size() * 2);
  Eigen::VectorXd landmarks, state;
  Eigen::Vector3d pose;
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
    state = this->_graph_slam_instance_->get_state_vector();
    landmarks = state.segment(3, state.size() - 3);
    pose = this->_pose_updater_->get_last_pose();  // get the last pose
    Eigen::Vector3d adjusted_pose = this->_pose_updater_->get_pose_at_timestamp(
        cones_timestamp);  // get the pose right before observation
    observations_global =
        common_lib::maths::local_to_global_coordinates(adjusted_pose, observations);
    initialization_time = rclcpp::Clock().now();
    covariance = this->_graph_slam_instance_->get_covariance_matrix();
    covariance_time = rclcpp::Clock().now();
  }

  // Data association
  Eigen::VectorXi associations = this->_data_association_->associate(
      landmarks, observations_global, covariance,
      observations_confidences);  // TODO: implement different mahalanobis distance
  this->_associations_ = associations;
  this->_observations_global_ = observations_global;
  this->_map_coordinates_ = state.segment(3, state.size() - 3);
  association_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Associations calculated");

  // Landmark filtering
  Eigen::VectorXd filtered_new_observations =
      this->_landmark_filter_->filter(observations_global, observations_confidences, associations);

  // Loop closure detection
  LoopClosure::Result result = _loop_closure_->detect(pose, landmarks, associations, observations);
  if (result.detected) {
    lap_counter_++;
    // Create a (hard-ish) lock on the landmarks' positions after the first lap.
    if (lap_counter_ == 1) {
      RCLCPP_INFO(rclcpp::get_logger("slam"),
                  "add_observations - First lap detected, locking landmarks");
      this->_graph_slam_instance_->lock_landmarks(0);
    }
    RCLCPP_INFO(rclcpp::get_logger("slam"), "Lap counter: %d", lap_counter_);
  }

  // Add observations to the graph
  {
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_observations - Mutex locked - processing observations");
    std::unique_lock uniq_lock(this->_mutex_);

    ObservationData observation_data(std::make_shared<Eigen::VectorXd>(observations),
                                     std::make_shared<Eigen::VectorXi>(associations),
                                     std::make_shared<Eigen::VectorXd>(observations_global),
                                     std::make_shared<Eigen::VectorXd>(observations_confidences),
                                     cones_timestamp);
    if (this->_optimization_under_way_) {
      this->_observation_data_queue_.push(observation_data);
      RCLCPP_INFO(rclcpp::get_logger("slam"),
                  "add_observations - Optimization under way, pushing observation data");
    }
    // Update the pose in the graph, as the vehicle might have moved since the last time a factor
    // was added by the motion callback
    this->_add_motion_data_to_graph(this->_pose_updater_, this->_graph_slam_instance_, true);
    this->_graph_slam_instance_->process_observations(observation_data);
    this->_landmark_filter_->delete_landmarks(filtered_new_observations);
  }
  factor_graph_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Mutex unlocked - Factors added");

  // Optimize the graph if optimization is synchronous
  if (this->_params_.slam_optimization_mode_ == "sync-parallel") {
    this->_asynchronous_optimization_routine();
  } else if (this->_params_.slam_optimization_mode_ == "sync") {
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Mutex locked - optimizing graph");
    std::unique_lock uniq_lock(this->_mutex_);
    this->_graph_slam_instance_->optimize();
    optimization_time = rclcpp::Clock().now();
    this->_pose_updater_->update_pose(gtsam_pose_to_eigen(this->_graph_slam_instance_->get_pose()));
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Mutex unlocked - graph optimized");
  }

  // Timekeeping
  if (this->_execution_times_ == nullptr) {
    return;
  }
  this->_execution_times_->at(3) = (covariance_time - initialization_time).seconds() * 1000.0;
  this->_execution_times_->at(2) = (association_time - covariance_time).seconds() * 1000.0;
  this->_execution_times_->at(4) = (factor_graph_time - association_time).seconds() * 1000.0;
  if (this->_params_.slam_optimization_mode_ == "sync") {
    this->_execution_times_->at(5) = (optimization_time - factor_graph_time).seconds() * 1000.0;
  }
}

void GraphSLAMSolver::load_initial_state(const Eigen::VectorXd& map, const Eigen::Vector3d& pose) {
  this->_graph_slam_instance_->load_initial_state(map, pose, this->_params_.preloaded_map_noise_);
  this->_pose_updater_->update_pose(pose);  // Update the pose in the pose updater
}

void GraphSLAMSolver::_asynchronous_optimization_routine() {
  rclcpp::Time start_time = rclcpp::Clock().now();

  // Copy data to optimize
  std::shared_ptr<GraphSLAMInstance> graph_slam_instance_copy;
  std::shared_ptr<PoseUpdater> pose_updater_copy;
  {
    const std::shared_lock lock(this->_mutex_);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "_asynchronous_optimization_routine - Shared mutex accessed");
    if (!this->_graph_slam_instance_->new_observation_factors()) {
      return;
    }
    graph_slam_instance_copy = this->_graph_slam_instance_->clone();
    pose_updater_copy = this->_pose_updater_->clone();
  }
  {
    std::unique_lock lock(this->_mutex_);
    this->_optimization_under_way_ = true;
  }
  rclcpp::Time initialization_time = rclcpp::Clock().now();

  // Optimize the graph
  RCLCPP_DEBUG(rclcpp::get_logger("slam"),
               "_asynchronous_optimization_routine - Starting optimization");
  graph_slam_instance_copy->optimize();
  pose_updater_copy->update_pose(gtsam_pose_to_eigen(graph_slam_instance_copy->get_pose()));
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "_asynchronous_optimization_routine - Graph optimized");
  rclcpp::Time optimization_time = rclcpp::Clock().now();

  // Rebuild the graph with the missed data
  {
    std::unique_lock lock(this->_mutex_);
    this->_optimization_under_way_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "_asynchronous_optimization_routine - Mutex locked");
    while (this->_motion_data_queue_.size() > 0 || this->_observation_data_queue_.size() > 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                   "_asynchronous_optimization_routine - Rebuilding graph, motion data queue size: "
                   "%ld, observation data queue size: %ld",
                   this->_motion_data_queue_.size(), this->_observation_data_queue_.size());
      bool process_pose = this->_observation_data_queue_.size() <= 0 ||
                          (this->_motion_data_queue_.size() > 0 &&
                           this->_motion_data_queue_.front().timestamp_ <=
                               this->_observation_data_queue_.front().timestamp_);
      if (process_pose) {
        std::shared_ptr<V2PMotionModel> motion_model_ptr =
            this->_motion_data_queue_.front().type_ == MotionInputType::VELOCITIES
                ? this->_motion_model_
                : this->_odometry_model;
        pose_updater_copy->predict_pose(this->_motion_data_queue_.front(),
                                        motion_model_ptr);  // Predict the pose
        this->_add_motion_data_to_graph(pose_updater_copy, graph_slam_instance_copy);
        this->_motion_data_queue_.pop();
      } else {
        this->_add_motion_data_to_graph(
            pose_updater_copy,
            graph_slam_instance_copy);  // To update graph pose before connecting factors
        graph_slam_instance_copy->process_observations(this->_observation_data_queue_.front());
        this->_observation_data_queue_.pop();
      }
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
GraphSLAMSolver::get_map_estimate() {  // TODO: include the map updates in observation to save
  // time
  const std::shared_lock lock(this->_mutex_);
  rclcpp::Time current_time = rclcpp::Clock().now();
  const gtsam::Values& graph_values = this->_graph_slam_instance_->get_graph_values_reference();
  std::vector<common_lib::structures::Cone> map_estimate;
  map_estimate.reserve(this->_graph_slam_instance_->get_landmark_counter());
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
  rclcpp::Time end_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "get_map_estimate - Time: %f ms",
               (end_time - current_time).seconds() * 1000.0);
  return map_estimate;
}

common_lib::structures::Pose GraphSLAMSolver::get_pose_estimate() {
  // const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_);
  // const gtsam::Key key = gtsam::Symbol('x', this->_pose_counter_);
  // const gtsam::Matrix covariance = marginals.marginalCovariance(key);
  const std::shared_lock lock(this->_mutex_);
  Eigen::Vector3d pose_vector = this->_pose_updater_->get_last_pose();

  return common_lib::structures::Pose(pose_vector(0), pose_vector(1), pose_vector(2), 0.0, 0.0, 0.0,
                                      this->_pose_updater_->get_last_pose_update());
}

std::vector<common_lib::structures::Pose> GraphSLAMSolver::get_trajectory_estimate() {
  const std::shared_lock lock(this->_mutex_);
  std::vector<common_lib::structures::Pose> trajectory;
  const gtsam::Values& graph_values = this->_graph_slam_instance_->get_graph_values_reference();
  trajectory.reserve(this->_graph_slam_instance_->get_pose_counter());
  for (auto it = graph_values.begin(); it != graph_values.end(); ++it) {
    // Iterate through the poses in the _graph_values_
    if (gtsam::Symbol(it->key).chr() == 'x') {
      gtsam::Pose2 pose = it->value.cast<gtsam::Pose2>();
      trajectory.push_back(common_lib::structures::Pose(pose.x(), pose.y(), pose.theta()));
    }
  }
  return trajectory;
}

Eigen::MatrixXd GraphSLAMSolver::get_covariance() {
  const std::shared_lock lock(this->_mutex_);
  return this->_graph_slam_instance_->get_covariance_matrix();
}

Eigen::VectorXi GraphSLAMSolver::get_associations() const {
  const std::shared_lock lock(this->_mutex_);
  return this->_associations_;
}

Eigen::VectorXd GraphSLAMSolver::get_observations_global() const {
  const std::shared_lock lock(this->_mutex_);
  return this->_observations_global_;
}

Eigen::VectorXd GraphSLAMSolver::get_map_coordinates() const {
  const std::shared_lock lock(this->_mutex_);
  return this->_map_coordinates_;
}