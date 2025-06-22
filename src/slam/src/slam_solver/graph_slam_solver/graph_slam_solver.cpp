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
#include "slam_solver/graph_slam_solver/pose_updater/linear_fusion_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/utils.hpp"

GraphSLAMSolver::GraphSLAMSolver(const SLAMParameters& params,
                                 std::shared_ptr<DataAssociationModel> data_association,
                                 std::shared_ptr<V2PMotionModel> motion_model,
                                 std::shared_ptr<LandmarkFilter> landmark_filter,
                                 std::shared_ptr<std::vector<double>> execution_times,
                                 std::shared_ptr<LoopClosure> loop_closure)
    : SLAMSolver(params, data_association, motion_model, landmark_filter, execution_times,
                 loop_closure) {
  // TODO: transform into range and bearing noises
  this->_graph_slam_instance_ = std::make_shared<GraphSLAMInstance>(
      params, graph_slam_optimizer_constructors_map.at(params.slam_optimization_type_)(params));
}

void GraphSLAMSolver::init([[maybe_unused]] std::weak_ptr<rclcpp::Node> node) {
  // Create a timer for asynchronous optimization
  if (this->_params_.slam_optimization_mode_ != "async" ||
      this->_params_.slam_optimization_period_ <= 0.0) {
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

bool GraphSLAMSolver::_add_motion_data_to_graph(
    const std::shared_ptr<PoseUpdater> pose_updater,
    const std::shared_ptr<GraphSLAMInstance> graph_slam_instance, bool force_update) {
  Eigen::Vector3d pose_difference_vector = pose_updater->get_accumulated_pose_difference();

  if (double pose_difference_norm =
          ::sqrt(pow(pose_difference_vector(0), 2) + pow(pose_difference_vector(1), 2) +
                 pow(pose_difference_vector(2), 2));
      pose_difference_norm < this->_params_.slam_min_pose_difference_ && !force_update) {
    return false;
  }

  // TODO: add noise thingy
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pose_differences = {
      {pose_difference_vector, Eigen::Vector3d(0.1, 0.1, 0.1)}};
  if (auto pose_updater_ptr = std::dynamic_pointer_cast<LinearFusionPoseUpdater>(pose_updater)) {
    if (pose_updater_ptr->odometry_pose_difference_ready()) {
      pose_differences.push_back({pose_updater_ptr->get_accumulated_odometry_pose_difference(),
                                  Eigen::Vector3d(0.1, 0.1, 0.1)});
    }
  }
  graph_slam_instance->process_pose_differences(pose_differences, pose_updater->get_last_pose());
  pose_updater->update_pose(pose_updater->get_last_pose());
}

void GraphSLAMSolver::add_odometry(const common_lib::structures::Pose& odometry) {
  std::unique_lock lock(this->_mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - Mutex accessed");
  rclcpp::Time start_time, motion_model_time, factor_graph_time;
  MotionData odometry_data(std::make_shared<Eigen::VectorXd>(3), odometry.timestamp);
  odometry_data.motion_data_->head<3>() << odometry.position.x, odometry.position.y,
      odometry.orientation;
  start_time = rclcpp::Clock().now();
  if (this->_optimization_under_way_) {
    this->_motion_data_queue_.push(odometry_data);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_odometry - Optimization under way, pushing motion data");
  }

  this->_pose_updater_->predict_pose(odometry_data, this->_motion_model_);
  motion_model_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - Pose updated");

  if (!this->_add_motion_data_to_graph(this->_pose_updater_, this->_graph_slam_instance_)) {
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - No new pose added to the graph");
  }
  factor_graph_time = rclcpp::Clock().now();

  // Timekeeping
  if (this->_execution_times_ == nullptr) {
    return;
  }
  this->_execution_times_->at(9) = (motion_model_time - start_time).seconds() * 1000.0;
  this->_execution_times_->at(10) = (factor_graph_time - motion_model_time).seconds() * 1000.0;
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_odometry - Mutex unlocked");
}

void GraphSLAMSolver::add_velocities(const common_lib::structures::Velocities& velocities) {
  std::unique_lock lock(this->_mutex_);
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Mutex accessed");
  rclcpp::Time start_time, motion_model_time, factor_graph_time;
  MotionData velocities_data(std::make_shared<Eigen::VectorXd>(3), velocities.timestamp_);
  velocities_data.motion_data_->head<3>() << velocities.velocity_x, velocities.velocity_y,
      velocities.rotational_velocity;
  start_time = rclcpp::Clock().now();
  if (this->_optimization_under_way_) {
    this->_motion_data_queue_.push(velocities_data);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_velocities - Optimization under way, pushing motion data");
  }

  this->_pose_updater_->predict_pose(velocities_data, this->_motion_model_);
  motion_model_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Pose updated");

  if (!this->_add_motion_data_to_graph(this->_pose_updater_, this->_graph_slam_instance_)) {
    RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - No new pose added to the graph");
  }
  factor_graph_time = rclcpp::Clock().now();

  // Timekeeping
  if (this->_execution_times_ == nullptr) {
    return;
  }
  this->_execution_times_->at(9) = (motion_model_time - start_time).seconds() * 1000.0;
  this->_execution_times_->at(10) = (factor_graph_time - motion_model_time).seconds() * 1000.0;
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_velocities - Mutex unlocked");
}

// void GraphSLAMSolver::_add_observation_data_to_graph(const)

void GraphSLAMSolver::add_observations(const std::vector<common_lib::structures::Cone>& cones) {
  if (cones.empty()) {
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
    pose = this->_pose_updater_->get_last_pose();
    observations_global = common_lib::maths::local_to_global_coordinates(pose, observations);
    initialization_time = rclcpp::Clock().now();
    covariance = this->_graph_slam_instance_->get_covariance_matrix();
    covariance_time = rclcpp::Clock().now();
  }

  // Data association
  Eigen::VectorXi associations(cones.size());
  associations = this->_data_association_->associate(
      landmarks, observations_global, covariance,
      observations_confidences);  // TODO: implement different mahalanobis distance
  association_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Associations calculated");

  // Landmark filtering
  Eigen::VectorXd unfiltered_new_observations;
  Eigen::VectorXd unfiltered_new_observations_confidences;
  for (int i = 0; i < associations.size(); i++) {
    if (associations(i) == -1) {
      unfiltered_new_observations.conservativeResize(unfiltered_new_observations.size() + 2);
      unfiltered_new_observations(unfiltered_new_observations.size() - 2) =
          observations_global(i * 2);
      unfiltered_new_observations(unfiltered_new_observations.size() - 1) =
          observations_global(i * 2 + 1);
      unfiltered_new_observations_confidences.conservativeResize(
          unfiltered_new_observations_confidences.size() + 1);
      unfiltered_new_observations_confidences(unfiltered_new_observations_confidences.size() - 1) =
          observations_confidences(i);
      associations(i) = -2;  // Mark as not ready to be added to the graph
    }
  }
  Eigen::VectorXd filtered_new_observations = this->_landmark_filter_->filter(
      unfiltered_new_observations, unfiltered_new_observations_confidences);
  if (this->_mission_ != common_lib::competition_logic::Mission::NONE &&
      this->_mission_ != common_lib::competition_logic::Mission::SKIDPAD &&
      this->_mission_ != common_lib::competition_logic::Mission::ACCELERATION &&
      lap_counter_ == 0) {
    // Set the associations to -1 for the filtered observations
    for (int i = 0; i < filtered_new_observations.size() / 2; i++) {  // TODO: clean this up
      for (int j = 0; j < associations.size(); j++) {
        if (std::hypot(filtered_new_observations(i * 2) - observations_global(j * 2),
                       filtered_new_observations(i * 2 + 1) - observations_global(j * 2 + 1)) <
            common_lib::structures::Cone::equality_tolerance) {
          associations(j) = -1;
          break;
        }
      }
    }
  }

  // Loop closure detection
  LoopClosure::Result result = _loop_closure_->detect(pose, landmarks, associations, observations);
  if (result.detected) {
    lap_counter_++;
    RCLCPP_INFO(rclcpp::get_logger("slam"), "Lap counter: %d", lap_counter_);
  }

  {
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "add_observations - Mutex locked - processing observations");
    std::unique_lock uniq_lock(this->_mutex_);

    // Only proceed if the vehicle has moved
    if (!this->_graph_slam_instance_->new_pose_factors()) {
      return;
    }

    ObservationData observation_data(std::make_shared<Eigen::VectorXd>(observations),
                                     std::make_shared<Eigen::VectorXi>(associations),
                                     std::make_shared<Eigen::VectorXd>(observations_global),
                                     std::make_shared<Eigen::VectorXd>(observations_confidences),
                                     cones.at(0).timestamp);
    if (this->_optimization_under_way_) {
      this->_observation_data_queue_.push(observation_data);
      RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                   "add_observations - Optimization under way, pushing observation data");
    }
    // Update the pose in the graph, as the vehicle might have moved since the last time a factor
    // was added by the motion callback
    this->_add_motion_data_to_graph(this->_pose_updater_, this->_graph_slam_instance_);
    this->_graph_slam_instance_->process_observations(observation_data);
    this->_landmark_filter_->delete_landmarks(filtered_new_observations);
  }
  factor_graph_time = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("slam"), "add_observations - Mutex unlocked - Factors added");

  // Optimize the graph
  if (this->_params_.slam_optimization_mode_ == "sync") {  // If optimization is synchronous
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

void GraphSLAMSolver::_asynchronous_optimization_routine() {  // TODO: maybe one day make a copy of
                                                              // the entire solver, it is cleaner
  rclcpp::Time start_time = rclcpp::Clock().now();
  std::shared_ptr<GraphSLAMInstance> graph_slam_instance_copy;
  std::shared_ptr<PoseUpdater> pose_updater_copy;
  {
    const std::shared_lock lock(this->_mutex_);
    RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                 "_asynchronous_optimization_routine - Shared mutex accessed");
    if (!this->_graph_slam_instance_->new_observation_factors()) {
      return;
    }
    graph_slam_instance_copy = this->_graph_slam_instance_;
    pose_updater_copy = std::make_shared<PoseUpdater>(*this->_pose_updater_);
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
        std::shared_ptr<V2PMotionModel> motion_model_ptr =
            this->_motion_data_queue_.front().type_ == MotionInputType::VELOCITIES
                ? this->_motion_model_
                : this->_odometry_model;
        pose_updater_copy->predict_pose(this->_motion_data_queue_.front(),
                                        motion_model_ptr);  // Predict the pose
        this->_add_motion_data_to_graph(
            pose_updater_copy, std::make_shared<GraphSLAMInstance>(graph_slam_instance_copy));
        this->_motion_data_queue_.pop();
      } else {
        this->_add_motion_data_to_graph(
            pose_updater_copy, std::make_shared<GraphSLAMInstance>(graph_slam_instance_copy));
        graph_slam_instance_copy->process_observations(this->_observation_data_queue_.front());
        this->_observation_data_queue_.pop();
      }
      // Print number of landmarks and poses
      RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                   "_asynchronous_optimization_routine - Number of landmarks: %d",
                   graph_slam_instance_copy->get_landmark_counter());
      RCLCPP_DEBUG(rclcpp::get_logger("slam"),
                   "_asynchronous_optimization_routine - Number of poses: %d",
                   graph_slam_instance_copy->get_pose_counter());
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
  return map_estimate;
}

common_lib::structures::Pose GraphSLAMSolver::get_pose_estimate() {
  // const gtsam::Marginals marginals(this->_factor_graph_, this->_graph_values_);
  // const gtsam::Key key = gtsam::Symbol('x', this->_pose_counter_);
  // const gtsam::Matrix covariance = marginals.marginalCovariance(key);

  Eigen::Vector3d pose_vector = this->_pose_updater_->get_last_pose();

  return common_lib::structures::Pose(pose_vector(0), pose_vector(1), pose_vector(2), 0.0, 0.0, 0.0,
                                      this->_pose_updater_->get_last_pose_update());
}

Eigen::MatrixXd GraphSLAMSolver::get_covariance() {
  return this->_graph_slam_instance_->get_covariance_matrix();
}
