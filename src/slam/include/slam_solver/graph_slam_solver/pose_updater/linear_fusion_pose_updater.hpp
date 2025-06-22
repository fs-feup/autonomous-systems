#pragma once

#include <Eigen/Dense>
#include <utility>

#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"

/**
 * @brief Class to update the pose of the vehicle
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose
 */
class LinearFusionPoseUpdater : public PoseUpdater {
  bool _received_first_velocities_ =
      false;  //< Flag to check if the first velocities have been received
  bool _received_first_odometry_ = false;  //< Flag to check if the first odometry has been received

  Eigen::Vector3d _accumulated_odometry_pose_difference_ =
      Eigen::Vector3d::Zero();  ///< Accumulated odometry pose difference since the last pose update

  Eigen::Vector3d _last_odometry_pose_ = Eigen::Vector3d::Zero();    //< Last odometry pose
  Eigen::Vector3d _last_velocities_pose_ = Eigen::Vector3d::Zero();  //< Last velocities pose
  Eigen::Vector3d _second_to_last_pose_ =
      Eigen::Vector3d::Zero();  //< Second to last pose, used for averaging
  rclcpp::Time _last_velocities_pose_update_ =
      rclcpp::Time(0);  //< Last time the velocities pose was updated

  bool _waiting_to_calculate_average_ = false;  //< Flag to check if we are waiting for odometry

  std::pair<Eigen::Vector3d, Eigen::Vector3d> update_pose_with_velocities(
      const MotionData& motion_data, std::shared_ptr<V2PMotionModel> motion_model);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> update_pose_with_odometry(
      const MotionData& motion_data, std::shared_ptr<V2PMotionModel> motion_model);

public:
  LinearFusionPoseUpdater() = default;

  LinearFusionPoseUpdater(const LinearFusionPoseUpdater& other);

  LinearFusionPoseUpdater& operator=(const LinearFusionPoseUpdater& other);

  /**
   * @brief Updates the last pose and returns the pose difference
   *
   * @param motion_data Motion data containing the velocities and timestamp
   * @param motion_model Motion model to apply the velocities
   */
  void predict_pose(const MotionData& motion_data,
                    std::shared_ptr<V2PMotionModel> motion_model) override;

  Eigen::Vector3d get_accumulated_odometry_pose_difference() const {
    return _accumulated_odometry_pose_difference_;
  }

  bool odometry_pose_difference_ready() const { return !_waiting_to_calculate_average_; }
};
