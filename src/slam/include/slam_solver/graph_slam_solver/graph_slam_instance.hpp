#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "common_lib/structures/circular_buffer.hpp"
#include "slam_config/general_config.hpp"
#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"
#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"

/**
 * @brief Graph SLAM instance class - class to hold the factor graph and the values
 * @details This class is used to hold the factor graph and the values for the graph SLAM solver
 * All operations to the factor graph and values are carried out by this object
 */
class GraphSLAMInstance {
protected:
  struct TimedPose {
    unsigned int pose_id;
    rclcpp::Time timestamp;
  };

  gtsam::NonlinearFactorGraph
      _factor_graph_;  //< Factor graph for the graph SLAM solver (only factors, no estimates)
  gtsam::Values
      _graph_values_;  //< Estimate for the graph SLAM solver: l_ are landmarks, x_ are poses
  unsigned int _pose_counter_ = 0;      //< Counter for the pose symbols
  unsigned int _landmark_counter_ = 0;  //< Counter for the landmark symbols
  bool _new_pose_node_ =
      false;  //< Flag to check if the pose was updated before adding observations
  bool _new_observation_factors_ =
      false;  //< Flag to check if new factors were added to the graph for optimization

  SLAMParameters _params_;                     //< Parameters for the SLAM solver
  std::shared_ptr<BaseOptimizer> _optimizer_;  //< Optimizer for the graph SLAM solver

  CircularBuffer<TimedPose>
      _pose_timestamps_;  //< Buffer to hold the timestamps of the poses added to the graph

  unsigned int get_landmark_id_at_time(const rclcpp::Time& timestamp) const;

public:
  GraphSLAMInstance() = default;

  GraphSLAMInstance(const SLAMParameters& params, std::shared_ptr<BaseOptimizer> optimizer);

  GraphSLAMInstance(const GraphSLAMInstance& other);

  GraphSLAMInstance& operator=(const GraphSLAMInstance& other);

  ~GraphSLAMInstance() = default;

  /**
   * @brief Clone the graph SLAM instance
   * @details This method is used to create a copy of the graph SLAM instance
   * It is useful for polymorphic classes that use pointers
   *
   * @return A shared pointer to the cloned graph SLAM instance
   */
  std::shared_ptr<GraphSLAMInstance> clone() const {
    return std::make_shared<GraphSLAMInstance>(*this);
  }

  /**
   * @brief Checks if new observation factors should be added
   *
   * @return true if new pose factors exist
   */
  bool new_pose_factors() const;

  /**
   * @brief Checks if it is worth running optimization
   *
   * @return true if new observation factors exist
   */
  bool new_observation_factors() const;

  /**
   * @brief Get the number of landmarks
   * @return unsigned int number of landmarks
   */
  unsigned int get_landmark_counter() const;

  /**
   * @brief Get the number of poses
   * @return unsigned int number of poses
   */
  unsigned int get_pose_counter() const;

  /**
   * @brief Get the current pose estimate
   *
   * @return const gtsam::Pose2 current pose
   */
  const gtsam::Pose2 get_pose() const;

  /**
   * @brief Get the graph values reference
   *
   * @return const gtsam::gtsam::Values& reference to the factor graph
   */
  const gtsam::Values& get_graph_values_reference() const;

  /**
   * @brief Get the state vector, EKF style
   *
   * @return const Eigen::VectorXd state vector
   */
  Eigen::VectorXd get_state_vector() const;

  /**
   * @brief Get the covariance matrix of the graph
   *
   * @return Eigen::MatrixXd covariance matrix
   */
  Eigen::MatrixXd get_covariance_matrix() const;

  /**
   * @brief Add observation factors to the graph
   *
   * @param observation_data Observation data to add to the graph
   */
  void process_observations(const ObservationData& observation_data);

  /**
   * @brief Loads a map and an initial pose into the graph
   *
   * @param map Coordinates of the map landmarks in the form of an Eigen vector [x1, y1, x2, y2,
   * ...]
   * @param pose Initial pose of the robot in the form of an Eigen vector [x, y, theta]
   * @param preloaded_map_noise Noise to apply to the preloaded map landmarks
   */
  void load_initial_state(const Eigen::VectorXd& map, const Eigen::Vector3d& pose,
                          double preloaded_map_noise);
  /**
   * @brief Process a new pose and respective pose difference
   * @details This method adds a new pose node to the graph values and calls the
   * process_pose_difference method to add the respective pose difference factor to the graph
   * @param pose_difference Pose difference from the last pose
   * @param noise_vector Noise associated with the pose difference
   * @param new_pose The new pose to add to the graph
   */
  void process_new_pose(const Eigen::Vector3d& pose_difference, const Eigen::Vector3d& noise_vector,
                        const Eigen::Vector3d& new_pose, const rclcpp::Time& pose_timestamp);

  /**
   * @brief Process a pose difference and add the respective factor to the graph
   * @param pose_difference Pose difference from the last pose
   * @param noise_vector Noise associated with the pose difference
   * @param before_pose_id ID of the pose before the difference (optional)
   * @param after_pose_id ID of the pose after the difference (optional)
   */
  void process_pose_difference(const Eigen::Vector3d& pose_difference,
                               const Eigen::Vector3d& noise_vector, unsigned int before_pose_id,
                               unsigned int after_pose_id);

  /**
   * @brief Process a pose difference and add the respective factor to the graph (after pose is
   * current pose)
   * @param pose_difference Pose difference from the last pose
   * @param noise_vector Noise associated with the pose difference
   * @param before_pose_id ID of the pose before the difference (optional)
   */
  void process_pose_difference(const Eigen::Vector3d& pose_difference,
                               const Eigen::Vector3d& noise_vector, unsigned int before_pose_id);

  /**
   * @brief Process a pose difference and add the respective factor to the graph (using current
   * pose and last graphed pose)
   * @param pose_difference Pose difference from the last pose
   * @param noise_vector Noise associated with the pose difference
   */
  void process_pose_difference(const Eigen::Vector3d& pose_difference,
                               const Eigen::Vector3d& noise_vector);

  void ensure_all_keys_have_initials();

  /**
   * @brief Runs optimization on the graph
   * @details This method calls the optimizer to run optimization on the current factor graph
   */
  void optimize();

  /**
   * @brief Soft locks the positions of the landmarks, can be used after the first lap
   *
   */
  void lock_landmarks(double locked_landmark_noise);
};