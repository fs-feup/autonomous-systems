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
  gtsam::Values _graph_values_;         //< Estimate for the graph SLAM solver
  unsigned int _pose_counter_ = 0;      //< Counter for the pose symbols
  unsigned int _landmark_counter_ = 0;  //< Counter for the landmark symbols

  /**
   * @brief Optimize the graph
   */
  void _optimize();

  /**
   * @brief Get the covariance matrix of the graph
   *
   * @return Eigen::MatrixXd covariance matrix
   */
  Eigen::MatrixXd _get_covariance();

  // friend class GraphSLAMSolver_BaseTest;
  friend class GraphSlamSolverTest_Prediction_1_Test;
  friend class GraphSlamSolverTest_Prediction_2_Test;
  friend class GraphSlamSolverTest_Prediction_3_Test;

public:
  /**
   * @brief Construct a new GraphSLAMSolver object
   *
   * @param params Parameters for the SLAM solver
   * @param data_association Data association module
   * @param motion_model Motion model
   */
  GraphSLAMSolver(const SLAMParameters& params,
                  std::shared_ptr<DataAssociationModel> data_association,
                  std::shared_ptr<V2PMotionModel> motion_model);

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
};