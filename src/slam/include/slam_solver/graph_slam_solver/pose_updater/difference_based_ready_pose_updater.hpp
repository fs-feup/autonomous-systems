#pragma once
#pragma once

#include <Eigen/Dense>
#include <utility>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "slam_config/general_config.hpp"
#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"

/**
 * @brief Class to update the pose of the vehicle, including a method to check if the pose is ready
 * for graph update depending on the accumulated pose difference
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose. The pose is considered ready for graph update if the accumulated pose difference is greater
 * than a minimum threshold
 */
class DifferenceBasedReadyPoseUpdater : public PoseUpdater {
  double minimum_pose_difference_ = 0.3;

public:
  explicit DifferenceBasedReadyPoseUpdater(const SLAMParameters& params);

  DifferenceBasedReadyPoseUpdater(const DifferenceBasedReadyPoseUpdater& other);

  DifferenceBasedReadyPoseUpdater& operator=(const DifferenceBasedReadyPoseUpdater& other);

  virtual ~DifferenceBasedReadyPoseUpdater() = default;

  /**
   * @brief Clones the PoseUpdater instance
   */
  virtual std::shared_ptr<PoseUpdater> clone() const = 0;

  /**
   * @brief Updates the last pose and returns the pose difference
   *
   * @param motion_data Motion data containing the velocities and timestamp
   * @param motion_model Motion model to apply the velocities
   */
  virtual void predict_pose(const MotionData& motion_data,
                            std::shared_ptr<V2PMotionModel> motion_model) = 0;

  /**
   * @brief Check if the accumulated pose difference is greater than the minimum threshold
   * @return true if the pose is ready for graph update, false otherwise
   */
  virtual bool pose_ready_for_graph_update() const override;
};