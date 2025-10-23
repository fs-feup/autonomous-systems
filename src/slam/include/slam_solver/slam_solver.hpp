#pragma once
#include <memory>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/structures/velocities.hpp"
#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"
#include "perception_sensor_lib/landmark_filter/base_landmark_filter.hpp"
#include "perception_sensor_lib/loop_closure/loop_closure.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_config/general_config.hpp"
#include "track_loader/track_loader.hpp"

/**
 * @brief Interface for SLAM solvers
 *
 * This class defines the interface for SLAM solvers like EKF or GTSAM
 */
class SLAMSolver {
protected:
  SLAMParameters _params_;
  std::shared_ptr<DataAssociationModel> _data_association_;  //< Data association pointer
  std::shared_ptr<V2PMotionModel> _motion_model_;            //< Motion model pointer
  std::shared_ptr<LandmarkFilter> _landmark_filter_;         //< Landmark filter pointer
  common_lib::competition_logic::Mission _mission_ = common_lib::competition_logic::Mission::NONE;
  std::shared_ptr<std::vector<double>>
      _execution_times_;                        //< Execution times: 0 -> total motion; 1 -> total
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
             std::shared_ptr<LandmarkFilter> landmark_filter,
             std::shared_ptr<std::vector<double>> execution_times,
             std::shared_ptr<LoopClosure> loop_closure);

  virtual ~SLAMSolver() = default;

  /**
   * @brief Add observations to the solver (correction step)
   *
   * @param cones Positions of the observations
   */
  virtual void add_observations(const std::vector<common_lib::structures::Cone>& cones) = 0;

  /**
   * @brief Loads a previously saved map and pose into the solver
   *
   * @param map coordinates of the landmarks in the form of [x1, y1, x2, y2, ...] in the global
   * frame
   * @param pose initial pose of the robot in the form of [x, y, theta] in the global frame
   */
  virtual void load_initial_state(const Eigen::VectorXd& map, const Eigen::Vector3d& pose) = 0;

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

  virtual Eigen::VectorXi get_associations() const = 0;

  virtual Eigen::VectorXd get_observations_global() const = 0;

  virtual Eigen::VectorXd get_map_coordinates() const = 0;

  /**
   * @brief Set the mission
   *
   * @param mission
   */
  void set_mission(common_lib::competition_logic::Mission mission);
};