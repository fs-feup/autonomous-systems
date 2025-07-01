#pragma once

#include <Eigen/Dense>
#include <utility>

#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/difference_based_ready_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/pose_updater_traits/second_pose_input_trait.hpp"

/**
 * @brief Class to update the pose of the vehicle
 * @details This class is the one to apply the motion model and keep track of the most up to date
 * pose
 */
class DoublePoseUpdater : public DifferenceBasedReadyPoseUpdater, public SecondPoseInputTrait {
  bool _received_first_velocities_ =
      false;  //< Flag to check if the first velocities have been received
  bool _received_first_odometry_ = false;  //< Flag to check if the first odometry has been received

  rclcpp::Time _last_velocities_receive_time_ =
      rclcpp::Time(0);  ///< Last pose update timestamp for velocities

  Eigen::Vector3d _accumulated_odometry_pose_difference_ =
      Eigen::Vector3d::Zero();  ///< Accumulated odometry pose difference since the last pose update

  Eigen::Vector3d _last_odometry_pose_ = Eigen::Vector3d::Zero();  //< Last odometry pose

public:
  explicit DoublePoseUpdater(const SLAMParameters& params);

  DoublePoseUpdater(const DoublePoseUpdater& other);

  DoublePoseUpdater& operator=(const DoublePoseUpdater& other);

  std::shared_ptr<PoseUpdater> clone() const override {
    return std::make_shared<DoublePoseUpdater>(*this);
  }

  /**
   * @brief Updates the last pose and returns the pose difference
   *
   * @param motion_data Motion data containing the velocities and timestamp
   * @param motion_model Motion model to apply the velocities
   */
  void predict_pose(const MotionData& motion_data,
                    std::shared_ptr<V2PMotionModel> motion_model) override;

  Eigen::Vector3d get_second_accumulated_pose_difference() const override;

  bool second_pose_difference_ready() const override;
};
