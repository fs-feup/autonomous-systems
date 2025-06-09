#pragma once
#include <memory>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/structures/velocities.hpp"
#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"
#include "perception_sensor_lib/loop_closure/loop_closure.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_config/general_config.hpp"

/**
 * @brief Interface for SLAM solvers
 *
 * This class defines the interface for SLAM solvers like EKF or GTSAM
 */
class SLAMSolver {
protected:
  SLAMParameters _params_;
  std::shared_ptr<DataAssociationModel> _data_association_;
  std::shared_ptr<V2PMotionModel> _motion_model_;
  std::shared_ptr<std::vector<double>>
      _execution_times_;  //< Execution times: 0 -> total motion; 1 -> total
                          // observation; the rest are solver specific
  std::shared_ptr<LoopClosure> _loop_closure_;  //< Loop closure object pointer

  rclcpp::Time _last_pose_update_ = rclcpp::Time(0);
  rclcpp::Time _last_observation_update_ = rclcpp::Time(0);

  bool _received_first_velocities_ =
      false;  //< Flag to check if the first velocities have been received
  
  
  int lap_counter_ = 0;  //< Lap counter for the graph SLAM solver

public:
  /**
   * @brief Construct a new SLAMSolver object
   *
   * @param params Parameters for the SLAM solver
   * @param data_association Data association module
   * @param motion_model Motion model
   * @param execution_times Timekeeping array
   * @param loop_closure Loop closure model
   */
  SLAMSolver(const SLAMParameters& params, std::shared_ptr<DataAssociationModel> data_association,
             std::shared_ptr<V2PMotionModel> motion_model,
             std::shared_ptr<std::vector<double>> execution_times, 
             std::shared_ptr<LoopClosure> loop_closure);

  virtual ~SLAMSolver() = default;

  /**
   * @brief Initialize the SLAM solver
   * @description This method is used to initialize the SLAM solver's
   * aspects that require the node e.g. timer callbacks
   *
   * @param node ROS2 node
   */
  virtual void init([[maybe_unused]] std::weak_ptr<rclcpp::Node> node) = 0;

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

  /**
   * @brief Get covariance matrix
   *
   * @return Eigen::MatrixXd
   */
  virtual Eigen::MatrixXd get_covariance() = 0;

  /**
   * @brief Get the lap counter
   *
   * @return int lap counter
   */
  virtual int get_lap_counter() = 0;
};