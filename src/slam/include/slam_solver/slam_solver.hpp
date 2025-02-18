#pragma once
#include <memory>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/structures/velocities.hpp"
#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"
#include "rclcpp/rclcpp.hpp"

struct SLAMSolverParameters {};

/**
 * @brief Interface for SLAM solvers
 *
 * This class defines the interface for SLAM solvers like EKF or GTSAM
 */
class SLAMSolver {
protected:
  std::shared_ptr<DataAssociationModel> _data_association_;
  std::shared_ptr<V2PMotionModel> _motion_model_;

  rclcpp::Time _last_pose_update_ = rclcpp::Time(0);
  rclcpp::Time _last_observation_update_ = rclcpp::Time(0);

public:
  /**
   * @brief Construct a new SLAMSolver object
   *
   * @param params Parameters for the SLAM solver
   * @param data_association Data association module
   * @param motion_model Motion model
   */
  SLAMSolver(const SLAMSolverParameters& params,
             std::shared_ptr<DataAssociationModel> data_association,
             std::shared_ptr<V2PMotionModel> motion_model);

  virtual ~SLAMSolver() = default;

  /**
   * @brief Add motion prior to the solver (prediction step)
   *
   * @param velocities Velocities of the robot
   */
  virtual void add_motion_prior(const common_lib::structures::Velocities& velocities) = 0;

  /**
   * @brief Add observations to the solver (correction step)
   *
   * @param cones Positions of the observations
   */
  virtual void add_observations(const std::vector<common_lib::structures::Cone>& cones) = 0;

  /**
   * @brief Get the map estimate object
   *
   * @return std::vector<common_lib::structures::Cone>
   */
  virtual std::vector<common_lib::structures::Cone> get_map_estimate() = 0;

  /**
   * @brief Get the pose estimate object
   *
   * @return common_lib::structures::Pose
   */
  virtual common_lib::structures::Pose get_pose_estimate() = 0;
};