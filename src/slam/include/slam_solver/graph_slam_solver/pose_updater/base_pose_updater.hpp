#pragma once

#include <Eigen/Dense>
#include <utility>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "slam_config/general_config.hpp"
#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"

/**
 * @brief Class to update the pose of the vehicle
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose. It allows for the pose estimate to be calculated independently of the graph optimization,
 * and only update the graph when needed.
 */
class PoseUpdater {
protected:
  Eigen::Vector3d _last_pose_;  ///< Last estimated pose (x, y, theta)
  Eigen::Vector3d _last_graphed_pose_ = Eigen::Vector3d::Zero();  ///< Last pose added to the graph
  Eigen::Matrix3d _last_pose_covariance_ =
      Eigen::Matrix3d::Zero();  ///< Covariance of the pose difference up to the last pose
  rclcpp::Time _last_pose_update_ = rclcpp::Time(0);  ///< Timestamp of the last pose update

  bool _received_first_motion_data_ =
      false;  ///< Flag to indicate if the first motion data has been received
  bool _new_pose_from_graph_ = false;  ///< Flag to indicate if the pose was updated from the graph

public:
  explicit PoseUpdater(const SLAMParameters& params);

  PoseUpdater(const PoseUpdater& other);

  PoseUpdater& operator=(const PoseUpdater& other);

  virtual ~PoseUpdater();

  /**
   * @brief Clone the pose updater
   * @details This method is used to create a copy of the pose updater
   * It is useful for polymorphic classes that use pointers to base class
   *
   * @return A shared pointer to the cloned pose updater
   */
  virtual std::shared_ptr<PoseUpdater> clone() const;

  /**
   * @brief Updates the last pose and returns the pose difference
   *
   * @param motion_data Motion data containing the velocities and timestamp
   * @param motion_model Motion model to apply the velocities
   */
  virtual void predict_pose(const MotionData& motion_data,
                            std::shared_ptr<V2PMotionModel> motion_model);

  /**
   * @brief Updates the last pose with the given pose
   * @details This method is used to set the last pose directly, for example after an optimization
   * or when the pose is known from another source. It resets the accumulated pose difference.
   *
   * @param last_pose The last pose to set
   */
  virtual void update_pose(const Eigen::Vector3d& last_pose);

  /**
   * @brief Check if the pose is ready for graph update
   * @return true if there is a new pose different from the graph, false otherwise
   */
  virtual bool pose_ready_for_graph_update() const;

  /** @brief Get the adjoint operator matrix for a given pose
   *  @details The adjoint operator matrix is used to transform the covariance
   *  of the pose difference when applying the motion model. It accounts for the
   *  rotation and translation of the pose.
   *  @param x_translation The x translation
   *  @param y_translation The y translation
   *  @param rotation_angle The rotation angle
   *  @return The adjoint operator matrix
   */
  Eigen::Matrix3d get_adjoint_operator_matrix(const double x_translation,
                                              const double y_translation,
                                              const double rotation_angle) const;

  Eigen::Vector3d get_last_pose() const { return _last_pose_; }

  Eigen::Vector3d get_last_graphed_pose() const { return _last_graphed_pose_; }

  /**
   * @brief Get the pose difference noise as standard deviation
   * @return The pose difference noise (standard deviation for x, y, theta)
   */
  Eigen::Vector3d get_pose_difference_noise() const {
    // Return the square root of the diagonal as standard deviation
    return _last_pose_covariance_.diagonal().array().sqrt();
  }

  rclcpp::Time get_last_pose_update() const { return _last_pose_update_; }
};