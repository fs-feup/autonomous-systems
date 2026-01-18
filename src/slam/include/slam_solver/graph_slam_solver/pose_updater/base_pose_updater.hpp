#pragma once

#include <Eigen/Dense>
#include <memory>
#include <utility>

#include "common_lib/structures/circular_buffer.hpp"
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
  struct TimedPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d pose;
    rclcpp::Time timestamp;

    TimedPose() : pose(Eigen::Vector3d::Zero()), timestamp(rclcpp::Time(0)) {}
    TimedPose(const Eigen::Vector3d& p, const rclcpp::Time& t) : pose(p), timestamp(t) {}
  };

  Eigen::Vector3d _last_pose_;
  Eigen::Vector3d _last_graphed_pose_;
  Eigen::Matrix3d _last_pose_covariance_;
  rclcpp::Time _last_pose_update_;

  bool _received_first_motion_data_ = false;
  bool _new_pose_from_graph_ = false;

  CircularBuffer<TimedPose> _pose_buffer_;

public:
  explicit PoseUpdater(const SLAMParameters& params);
  PoseUpdater(const PoseUpdater& other);
  PoseUpdater& operator=(const PoseUpdater& other);
  virtual ~PoseUpdater();

  /**
   * @brief Class to update the pose of the vehicle
   * @details This class is the one to apply the motion model and keep track of the most up to date
   * pose. It allows for the pose estimate to be calculated independently of the graph optimization,
   * and only update the graph when needed.
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

  Eigen::Matrix3d get_adjoint_operator_matrix(double x_translation, double y_translation,
                                              double rotation_angle) const;

  Eigen::Vector3d get_last_pose() const { return _last_pose_; }

  Eigen::Vector3d get_last_graphed_pose() const { return _last_graphed_pose_; }

  Eigen::Vector3d get_pose_difference_noise() const {
    // Return the square root of the diagonal as standard deviation
    return _last_pose_covariance_.diagonal().array().sqrt();
  }

  rclcpp::Time get_last_pose_update() const { return _last_pose_update_; }

  inline size_t pose_buffer_size() const { return _pose_buffer_.size(); }
  inline size_t pose_buffer_capacity() const { return _pose_buffer_.capacity(); }

  Eigen::Vector3d get_pose_at_timestamp(const rclcpp::Time& timestamp) const;
};
