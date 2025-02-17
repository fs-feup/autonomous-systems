#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "slam_solver/slam_solver.hpp"

/**
 * @brief Graph SLAM solver class
 */
class GraphSLAMSolver : public SLAMSolver {
  gtsam::NonlinearFactorGraph
      _factor_graph_;  //< Factor graph for the graph SLAM solver (only factors, no estimates)
  gtsam::Values _graph_values_;         //< Estimate for the graph SLAM solver
  unsigned int _pose_counter_ = 0;      //< Counter for the pose symbols
  unsigned int _landmark_counter_ = 0;  //< Counter for the landmark symbols

public:
  /**
   * @brief Construct a new GraphSLAMSolver object
   *
   * @param params Parameters for the SLAM solver
   * @param data_association Data association module
   * @param motion_model Motion model
   */
  GraphSLAMSolver(const SLAMSolverParameters& params,
                  std::shared_ptr<DataAssociationModel> data_association,
                  std::shared_ptr<V2PMotionModel> motion_model);

  ~GraphSLAMSolver() = default;

  /**
   * @brief Add motion prior to the solver (prediction step)
   */
  void add_motion_prior(const common_lib::structures::Velocities& velocities) override;

  /**
   * @brief Add observation to the solver (correction step)
   */
  void add_observation(const common_lib::structures::Cone& position) override;

  /**
   * @brief Add observations to the solver (correction step)
   */
  void add_observations(const std::vector<common_lib::structures::Cone>& positions) override;

  /**
   * @brief Get the map estimate object
   */
  std::vector<common_lib::structures::Cone> get_map_estimate() override;

  /**
   * @brief Get the pose estimate object
   */
  common_lib::structures::Pose get_pose_estimate() override;

  // void addPose(const Pose2& pose) override;

  // void addLandmark(const Point2& landmark) override;

  // void addPriorPose(const Pose2& pose) override;

  // void addPriorLandmark(const Point2& landmark) override;

  // void addBetweenFactor(const Pose2& pose1, const Pose2& pose2, const Pose2& relative_pose)
  // override;

  // void addBearingRangeFactor(const Pose2& pose, const Point2& landmark, const BearingRange&
  // bearing_range) override;

  // void optimize() override;

  // Pose2 getPose(const size_t pose_id) const override;

  // Point2 getLandmark(const size_t landmark_id) const override;

  // Marginals getMarginals() const override;

  // void print() const override;
};