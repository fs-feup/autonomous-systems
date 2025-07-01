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
class VelocitiesBasedPoseUpdater : public DifferenceBasedReadyPoseUpdater {
  bool _received_first_velocities_ =
      false;  //< Flag to check if the first velocities have been received
  rclcpp::Time _last_velocities_receive_time_ = rclcpp::Time(0);  ///< Last pose update timestamp

public:
  explicit VelocitiesBasedPoseUpdater(const SLAMParameters& params);

  VelocitiesBasedPoseUpdater(const VelocitiesBasedPoseUpdater& other);

  VelocitiesBasedPoseUpdater& operator=(const VelocitiesBasedPoseUpdater& other);

  std::shared_ptr<PoseUpdater> clone() const override {
    return std::make_shared<VelocitiesBasedPoseUpdater>(*this);
  }

  /**
   * @brief Updates the last pose and returns the pose difference
   *
   * @param motion_data Motion data containing the velocities and timestamp
   * @param motion_model Motion model to apply the velocities
   */
  void predict_pose(const MotionData& motion_data,
                    std::shared_ptr<V2PMotionModel> motion_model) override;
};
