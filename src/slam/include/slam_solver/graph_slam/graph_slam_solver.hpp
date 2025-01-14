#pragma once

#include "slam_solver/slam_solver.hpp"

class GraphSLAMSolver : public SLAMSolver {
public:
  GraphSLAMSolver() = default;

  ~GraphSLAMSolver() = default;

  // void add_prior_pose(const common_lib::structures::Velocities& velocities) override;

  // void add_observation(const common_lib::structures::Position& position) override;

  // void solve() override;

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