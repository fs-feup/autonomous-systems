#pragma once
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/structures/velocities.hpp"
#include "slam_config/general_config.hpp"

/**
 * @brief Interface for SLAM solvers
 *
 * This class defines the interface for SLAM solvers like EKF or GTSAM
 */
class SLAMSolver {
public:
  /**
   * @brief Construct a new SLAMSolver object
   */
  SLAMSolver(const SLAMParameters& params){};

  virtual ~SLAMSolver() = default;

  /**
   * @brief Add motion prior to the solver (prediction step)
   */
  virtual void add_motion_prior(const common_lib::structures::Velocities& velocities) = 0;

  /**
   * @brief Add observation to the solver (correction step)
   */
  virtual void add_observations(const std::vector<common_lib::structures::Position>& positions) = 0;

  /**
   * @brief Get the map estimate object
   */
  virtual std::vector<common_lib::structures::Cone> get_map_estimate() = 0;

  /**
   * @brief Get the pose estimate object
   */
  virtual common_lib::structures::Pose get_pose_estimate() = 0;
};