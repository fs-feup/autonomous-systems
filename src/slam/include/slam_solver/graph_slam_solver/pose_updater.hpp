#pragma once

#include <Eigen/Dense>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"

/**
 * @brief Class to update the pose of the vehicle
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose
 */
class PoseUpdater {
  Eigen::Vector3d _last_pose_;
  rclcpp::Time _last_pose_update_ = rclcpp::Time(0);

  bool _received_first_velocities_ =
      false;  //< Flag to check if the first velocities have been received

public:
  PoseUpdater() = default;

  PoseUpdater(const PoseUpdater& other);

  PoseUpdater& operator=(const PoseUpdater& other);

  void update_pose(const MotionData& motion_data, std::shared_ptr<V2PMotionModel> motion_model);

  Eigen::Vector3d get_last_pose() const { return _last_pose_; }

  rclcpp::Time get_last_pose_update() const { return _last_pose_update_; }

  void set_last_pose(const Eigen::Vector3d& last_pose) { _last_pose_ = last_pose; }
};
