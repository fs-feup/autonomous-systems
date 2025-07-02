#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

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
  gtsam::NonlinearFactorGraph
      _factor_graph_;  //< Factor graph for the graph SLAM solver (only factors, no estimates)
  gtsam::Values
      _graph_values_;  //< Estimate for the graph SLAM solver: l_ are landmarks, x_ are poses
  Eigen::Vector3d _accumulated_pose_difference_ =
      Eigen::Vector3d::Zero();          //< Accumulated pose difference from the last pose node
  unsigned int _pose_counter_ = 0;      //< Counter for the pose symbols
  unsigned int _landmark_counter_ = 0;  //< Counter for the landmark symbols
  bool _new_pose_node_ =
      false;  //< Flag to check if the pose was updated before adding observations
  bool _new_observation_factors_ =
      false;  //< Flag to check if new factors were added to the graph for optimization

  SLAMParameters _params_;                     //< Parameters for the SLAM solver
  std::shared_ptr<BaseOptimizer> _optimizer_;  //< Optimizer for the graph SLAM solver

public:
  GraphSLAMInstance() = default;

  GraphSLAMInstance(const SLAMParameters& params, std::shared_ptr<BaseOptimizer> optimizer);

  GraphSLAMInstance(const GraphSLAMInstance& other);

  GraphSLAMInstance& operator=(const GraphSLAMInstance& other);

  ~GraphSLAMInstance() = default;

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

  unsigned int get_landmark_counter() const;

  unsigned int get_pose_counter() const;

  // TODO: improve copying times and referencing and stuff like that

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

  void process_new_pose(const Eigen::Vector3d& pose_difference, const Eigen::Vector3d& noise_vector,
                        const Eigen::Vector3d& new_pose);

  void process_pose_difference(const Eigen::Vector3d& pose_difference,
                               const Eigen::Vector3d& noise_vector, unsigned int before_pose_id,
                               unsigned int after_pose_id);

  void process_pose_difference(const Eigen::Vector3d& pose_difference,
                               const Eigen::Vector3d& noise_vector, unsigned int before_pose_id);

  void process_pose_difference(const Eigen::Vector3d& pose_difference,
                               const Eigen::Vector3d& noise_vector);

  /**
   * @brief Runs optimization on the graph
   */
  void optimize();

  /**
   * @brief Soft locks the positions of the landmarks, can be used after the first lap
   *
   */
  void lock_landmarks(double locked_landmark_noise);
};