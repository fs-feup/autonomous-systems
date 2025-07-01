#pragma once

#include <Eigen/Dense>
#include <utility>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/difference_based_ready_pose_updater.hpp"

/**
 * @brief Class to update the pose of the vehicle
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose
 */
class OdometryBasedPoseUpdater : public DifferenceBasedReadyPoseUpdater {
  bool _received_first_odometry_ =
      false;  ///< Flag to check if the first odometry has been received
  Eigen::Vector3d _last_odometry_pose_ = Eigen::Vector3d::Zero();  ///< Last odometry pose
public:
  explicit OdometryBasedPoseUpdater(const SLAMParameters& params);

  OdometryBasedPoseUpdater(const OdometryBasedPoseUpdater& other);

  OdometryBasedPoseUpdater& operator=(const OdometryBasedPoseUpdater& other);

  std::shared_ptr<PoseUpdater> clone() const override {
    return std::make_shared<OdometryBasedPoseUpdater>(*this);
  }

  /**
   * @brief Updates the last pose based on motion information
   *
   * @param motion_data Motion data containing the odometry and timestamp
   * @param motion_model Motion model to apply to the data
   */
  void predict_pose(const MotionData& motion_data,
                    std::shared_ptr<V2PMotionModel> motion_model) override;
};
