#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <queue>

#include "slam_solver/slam_solver.hpp"

/**
 * @brief Data structure to hold motion data
 * @details used to record the velocities received and to redo their processing after optimization
 */
struct MotionData {
  std::shared_ptr<Eigen::Vector3d> velocities_;
  rclcpp::Time timestamp_;

  MotionData() = default;
  MotionData(std::shared_ptr<Eigen::Vector3d> velocities, rclcpp::Time timestamp)
      : velocities_(velocities), timestamp_(timestamp) {}
};

/**
 * @brief Data structure to hold observation data
 * @details used to record the velocities received and to redo their processing after optimization
 */
struct ObservationData {
  std::shared_ptr<Eigen::VectorXd> observations_;
  std::shared_ptr<Eigen::VectorXi> associations_;
  std::shared_ptr<Eigen::VectorXd> observations_global_;
  rclcpp::Time timestamp_;

  ObservationData() = default;
  ObservationData(std::shared_ptr<Eigen::VectorXd> observations,
                  std::shared_ptr<Eigen::VectorXi> associations,
                  std::shared_ptr<Eigen::VectorXd> observations_global, rclcpp::Time timestamp)
      : observations_(observations),
        associations_(associations),
        observations_global_(observations_global),
        timestamp_(timestamp) {}
};

/**
 * @brief Class to update the pose of the vehicle
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose
 */
class PoseUpdater {
  Eigen::Vector3d _last_pose_;
  rclcpp::Time _last_pose_update_ = rclcpp::Time(0);

  bool _received_first_velocities_ =
      false;  //< Flag to check if the first velocities have been received

public:
  PoseUpdater() = default;

  PoseUpdater(const PoseUpdater& other);

  PoseUpdater& operator=(const PoseUpdater& other);

  void update_pose(const MotionData& motion_data, std::shared_ptr<V2PMotionModel> motion_model);

  Eigen::Vector3d get_last_pose() const { return _last_pose_; }

  rclcpp::Time get_last_pose_update() const { return _last_pose_update_; }

  void set_last_pose(const Eigen::Vector3d& last_pose) { _last_pose_ = last_pose; }
};

/**
 * @brief Graph SLAM instance class - class to hold the factor graph and the values
 * @details This class is used to hold the factor graph and the values for the graph SLAM solver
 * All operations to the factor graph and values are carried out by this object
 */
class GraphSLAMInstance {
  gtsam::NonlinearFactorGraph
      _factor_graph_;  //< Factor graph for the graph SLAM solver (only factors, no estimates)
  gtsam::Values _graph_values_;         //< Estimate for the graph SLAM solver
  unsigned int _pose_counter_ = 0;      //< Counter for the pose symbols
  unsigned int _landmark_counter_ = 0;  //< Counter for the landmark symbols
  bool _new_pose_node_ =
      false;  //< Flag to check if the pose was updated before adding observations
  bool _new_observation_factors_ =
      false;  //< Flag to check if new factors were added to the graph for optimization

  SLAMParameters _params_;  //< Parameters for the SLAM solver

public:
  GraphSLAMInstance() = default;

  GraphSLAMInstance(const SLAMParameters& params);

  GraphSLAMInstance(const GraphSLAMInstance& other);

  GraphSLAMInstance& operator=(const GraphSLAMInstance& other);

  ~GraphSLAMInstance() = default;

  /**
   * @brief Checks if new observation factors should be added
   *
   * @return true if new pose factors exist
   */
  bool should_process_observations() const;

  /**
   * @brief Checks if it is worth running optimization
   *
   * @return true if new observation factors exist
   */
  bool should_perform_optimization() const;

  unsigned int get_landmark_counter() const;

  unsigned int get_pose_counter() const;

  // TODO: improve copying times and referencing and stuff like that

  const gtsam::Pose2 get_pose() const;

  /**
   * @brief Get the graph values reference
   *
   * @return const gtsam::gtsam::Values& reference to the factor graph
   */
  const gtsam::Values& get_graph_values_reference() const;

  /**
   * @brief Get the state vector, EKF style
   *
   * @return const Eigen::VectorXd state vector
   */
  Eigen::VectorXd get_state_vector() const;

  /**
   * @brief Get the covariance matrix of the graph
   *
   * @return Eigen::MatrixXd covariance matrix
   */
  Eigen::MatrixXd get_covariance_matrix() const;

  /**
   * @brief Add observation factors to the graph
   *
   * @param observation_data Observation data to add to the graph
   */
  void process_observations(const ObservationData& observation_data);

  /**
   * @brief Add motion prior to the graph
   *
   * @param pose Pose to add to the graph
   */
  void process_pose(const gtsam::Pose2& pose);

  /**
   * @brief Runs optimization on the graph
   */
  void optimize();
};

/**
 * @brief Graph SLAM solver class
 *
 * @details This class implements the Graph SLAM solver using GTSAM
 * It uses a factor graph to represent the problem and the values to store the estimates
 * This specific class controls the access to the models used in the SLAM implementation,
 * including measures for parallel execution
 */
class GraphSLAMSolver : public SLAMSolver {
  GraphSLAMInstance _graph_slam_instance_;  //< Instance of the graph SLAM solver
  PoseUpdater _pose_updater_;               //< Pose updater for the graph SLAM solver
  std::queue<MotionData>
      _motion_data_queue_;  //< Queue of velocities received while optimization ran
  std::queue<ObservationData>
      _observation_data_queue_;  //< Queue of observations received while optimization ran
  rclcpp::TimerBase::SharedPtr _optimization_timer_;  //< Timer for asynchronous optimization
  std::shared_mutex _mutex_;  //< Mutex for the graph SLAM solver, locks access to the graph
  bool _optimization_under_way_ = false;  //< Flag to check if the optimization is under way

  rclcpp::CallbackGroup::SharedPtr
      _reentrant_group_;  //< Reentrant callback group for the timer callback

  /**
   * @brief Asynchronous optimization routine
   * @details This method is used to run the optimization in a separate thread
   * It is called by the timer and runs the optimization on the graph
   * It also updates the pose and the graph values accordingly afterwards
   */
  void _asynchronous_optimization_routine();

  friend class GraphSlamSolverTest_MotionAndObservation_Test;

public:
  /**
   * @brief Construct a new GraphSLAMSolver object
   *
   * @param params Parameters for the SLAM solver
   * @param data_association Data association module
   * @param motion_model Motion model
   * @param execution_times Timekeeping array
   */
  GraphSLAMSolver(const SLAMParameters& params,
                  std::shared_ptr<DataAssociationModel> data_association,
                  std::shared_ptr<V2PMotionModel> motion_model,
                  std::shared_ptr<std::vector<double>> execution_times);

  ~GraphSLAMSolver() = default;

  /**
   * @brief Initialize the SLAM solver
   * @description This method is used to initialize the SLAM solver's
   * aspects that require the node e.g. timer callbacks
   *
   * @param node ROS2 node
   */
  void init(std::weak_ptr<rclcpp::Node> node) override;

  /**
   * @brief Add motion prior to the solver (prediction step)
   */
  void add_motion_prior(const common_lib::structures::Velocities& velocities) override;

  /**
   * @brief Add observations to the solver (correction step)
   *
   * @param cones Positions of the observations
   */
  void add_observations(const std::vector<common_lib::structures::Cone>& cones) override;

  /**
   * @brief Get the map estimate object
   *
   * @return std::vector<common_lib::structures::Cone>
   */
  std::vector<common_lib::structures::Cone> get_map_estimate() override;

  /**
   * @brief Get the pose estimate object
   *
   *  @ return common_lib::structures::Pose
   */
  common_lib::structures::Pose get_pose_estimate() override;

  /**
   * @brief Get the covariance matrix of the graph
   *
   * @return Eigen::MatrixXd covariance matrix
   */
  Eigen::MatrixXd get_covariance() override;

  /**
   * Timekeeping array
   * - 0: total motion time (prediction)
   * - 1: total observation time (correction)
   * - 2: data association time
   * - 3: covariance time
   * - 4: factor graph time (in add_observations)
   * - 5: optimization time
   * - 6: optimization copy / initialization time
   * - 7: redo time in optimization routine
   * - 8: total asynchronous optimization routine time
   *
   */
};