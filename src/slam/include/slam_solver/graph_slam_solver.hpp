#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "slam_solver/slam_solver.hpp"

/**
 * @brief Graph SLAM solver class
 *
 * @details This class implements the Graph SLAM solver using GTSAM
 * It uses a factor graph to represent the problem and the values to store the estimates
 */
class GraphSLAMSolver : public SLAMSolver {
  gtsam::NonlinearFactorGraph
      _factor_graph_;  //< Factor graph for the graph SLAM solver (only factors, no estimates)
  gtsam::Values _graph_values_;          //< Estimate for the graph SLAM solver
  unsigned int _pose_counter_ = 0;       //< Counter for the pose symbols
  unsigned int _landmark_counter_ = 0;   //< Counter for the landmark symbols
  Eigen::MatrixXd _covariance_;          //< Covariance matrix of the graph
  bool _covariance_up_to_date_ = false;  //< Flag to check if the covariance matrix is up to date
  bool _updated_pose_ = false;  //< Flag to check if the pose was updated before adding observations
  bool _first_observation_ = true;  //< Flag to check if it is the first observation
  gtsam::Pose2
      _last_pose_;  //< Last calculated pose (from the motion model, different than the graph)
  rclcpp::TimerBase::SharedPtr _optimization_timer_;  //< Timer for asynchronous optimization

  /**
   * @brief Optimize the graph
   */
  void _optimize();

  /**
   * @brief Initialize the graph SLAM solver (called by both constructors)
   */
  void _init();

  friend class GraphSlamSolverTest_MotionAndObservation_Test;
  friend class GraphSlamSolverTest_Prediction_Test;

public:
  /**
   * @brief Construct a new GraphSLAMSolver object
   *
   * @param params Parameters for the SLAM solver
   * @param data_association Data association module
   * @param motion_model Motion model
   * @param execution_times Timekeeping array
   * @param node SLAM ROS2 node
   */
  GraphSLAMSolver(const SLAMParameters& params,
                  std::shared_ptr<DataAssociationModel> data_association,
                  std::shared_ptr<V2PMotionModel> motion_model,
                  std::shared_ptr<std::vector<double>> execution_times,
                  std::weak_ptr<rclcpp::Node> node);

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
   */
};