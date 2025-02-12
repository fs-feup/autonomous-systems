#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "slam_solver/slam_solver.hpp"

/**
 * @brief Graph SLAM solver class
 */
class GraphSLAMSolver : public SLAMSolver {
  gtsam::NonlinearFactorGraph
      _factor_graph_;  //< Factor graph for the graph SLAM solver (only factors, no estimates)
  gtsam::Values _graph_values_;  //< Estimate for the graph SLAM solver

public:
  GraphSLAMSolver();

  ~GraphSLAMSolver() = default;

  /**
   * @brief Add motion prior to the solver (prediction step)
   */
  void add_motion_prior(const common_lib::structures::Velocities& velocities) override;

  /**
   * @brief Add observation to the solver (correction step)
   */
  void add_observation(const common_lib::structures::Position& position) override;

  void solve();

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