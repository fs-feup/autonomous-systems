#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <perception_sensor_lib/loop_closure/lap_counter.hpp>
#include <queue>

#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"
#include "slam_solver/graph_slam_solver/graph_slam_instance.hpp"
#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"
#include "slam_solver/graph_slam_solver/pose_updater.hpp"
#include "slam_solver/slam_solver.hpp"

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
  Eigen::VectorXi _associations_;         //< Associations of the cones in the map
  Eigen::VectorXd _observations_global_;  //< Global observations of the cones
  Eigen::VectorXd _map_coordinates_;      //< Coordinates of the landmarks in the map

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
                  std::shared_ptr<LandmarkFilter> landmark_filter,
                  std::shared_ptr<std::vector<double>> execution_times,
                  std::shared_ptr<LoopClosure> loop_closure);

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
   * @brief Initialize the graph SLAM solver with a previously saved map and pose
   *
   * @param map Coordinates of the landmarks in the form of [x1, y1, x2, y2, ...] relative to the
   * global frame
   * @param pose Pose of the robot in the form of [x, y, theta] relative to the global frame
   */
  void load_initial_state(const Eigen::VectorXd& map, const Eigen::VectorXd& pose) override;

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
   * @brief Get the lap counter
   *
   * @return int lap counter
   */
  int get_lap_counter() override { return lap_counter_; }

  Eigen::VectorXi get_associations() const override;

  Eigen::VectorXd get_observations_global() const override;

  Eigen::VectorXd get_map_coordinates() const override;

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
   * - 9: motion model time
   * - 10: factor graph time (in add_motion_prior)
   */
};