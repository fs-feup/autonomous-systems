#pragma once

#include <Eigen/Dense>
#include <utility>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "slam_config/general_config.hpp"
#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"

/**
 * @brief Class to update the pose of the vehicle
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose
 */
class PoseUpdater {
protected:
  Eigen::Vector3d _last_pose_;
  rclcpp::Time _last_pose_update_ = rclcpp::Time(0);
  Eigen::Vector3d _accumulated_pose_difference_ =
      Eigen::Vector3d::Zero();  ///< Accumulated pose difference since the last pose update

  bool _new_pose_from_graph_ = false;  ///< Flag to indicate if the pose was updated from the graph

public:
  explicit PoseUpdater(const SLAMParameters& params);

  PoseUpdater(const PoseUpdater& other);

  PoseUpdater& operator=(const PoseUpdater& other);

  virtual ~PoseUpdater() = default;

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
   * @brief Updates the last pose with the given pose
   * @details This method is used to set the last pose directly, for example after an optimization
   * or when the pose is known from another source. It resets the accumulated pose difference.
   *
   * @param last_pose The last pose to set
   */
  void update_pose(const Eigen::Vector3d& last_pose);

  Eigen::Vector3d get_last_pose() const { return _last_pose_; }

  rclcpp::Time get_last_pose_update() const { return _last_pose_update_; }

  Eigen::Vector3d get_accumulated_pose_difference() const { return _accumulated_pose_difference_; }

  virtual bool pose_ready_for_graph_update() const { return _new_pose_from_graph_; }
};