#pragma once

#include <Eigen/Sparse>
#include <chrono>

#include "common_lib/conversions/cones.hpp"
#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/observation_model/base_observation_model.hpp"
#include "slam_solver/slam_solver.hpp"

class EKFSLAMSolver : public SLAMSolver {
  SLAMParameters slam_parameters_;
  std::shared_ptr<ObservationModel> observation_model_;
  Eigen::SparseMatrix<double> state_;
  Eigen::SparseMatrix<double> covariance_;
  Eigen::SparseMatrix<double> process_noise_matrix_;

  rclcpp::Time last_update_;

  bool velocities_received_ = false;
  bool cones_receieved_ = false;

  /**
   * @brief Get the observation noise matrix object used in the correction step of the EKF
   * with the right dimensions
   *
   * @param num_landmarks number of observed landmarks in a given EKF correction step
   * @return Eigen::SparseMatrix<double>
   */
  Eigen::SparseMatrix<double> get_observation_noise_matrix(int num_landmarks) const;

  /**
   * @brief executed when velocity data is received. Prediction step of the EKF
   * which is meant to capture changes in the state of the system
   *
   * @param state vector with position and orientation, followed by the landmark positions {car_x,
   * car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param covariance
   * @param process_noise_matrix estimated process noise
   * @param last_update last time velocity data was received
   * @param velocities new velocity data
   */
  void predict(Eigen::SparseMatrix<double>& state, Eigen::SparseMatrix<double>& covariance,
               const Eigen::SparseMatrix<double>& process_noise_matrix,
               const rclcpp::Time last_update,
               const common_lib::structures::Velocities& velocities);

  /**
   * @brief correction step of the EKF that updates the state and covariance based on the observed
   * landmarks
   *
   * @param state state vector previouslt to the correction step
   * @param covariance covariance matrix previously to the correction step
   * @param matched_landmarks_indices indexes of the x coordinate of landmarks in the state vector
   * that were observed
   * @param matched_observations landmarks in the state vector that were observed, in the form
   * {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   */
  void correct(Eigen::SparseMatrix<double>& state, Eigen::SparseMatrix<double>& covariance,
               const std::vector<int>& observed_landmarks_indices,
               const Eigen::SparseMatrix<double>& matched_observations);

  /**
   * @brief add new landmarks to the state vector and covariance matrix
   *
   * @param state state vector with the car's position and orientation, followed by the landmarks
   * @param covariance covariance matrix of the state vector
   * @param new_landmarks new landmarks in the form {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * in the car's frame
   * @param new_landmarks_confidence confidence of the new landmarks
   */
  void state_augmentation(Eigen::SparseMatrix<double>& state,
                          Eigen::SparseMatrix<double>& covariance,
                          const Eigen::SparseMatrix<double>& new_landmarks,
                          const Eigen::SparseMatrix<double>& new_landmarks_confidence);

  std::vector<common_lib::structures::Cone> get_map_estimate() override;
  common_lib::structures::Pose get_pose_estimate() override;

  /**
   * @brief update the process noise matrix so that its dimensions match the covariance matrix
   */
  void update_process_noise_matrix();

  friend class EKFSLAMSolverTest_stateAugmentation_Test;
  friend class EKFSLAMSolverTest_stateAugmentation2_Test;

public:
  EKFSLAMSolver(const SLAMParameters& params,
                std::shared_ptr<DataAssociationModel> data_association,
                std::shared_ptr<V2PMotionModel> motion_model);

  /**
   * @brief Executed to deal with new velocity data
   *
   * @param velocities
   */
  void add_motion_prior(const common_lib::structures::Velocities& velocities) override;

  /**
   * @brief process observations of landmarks
   *
   * @param position
   */
  void add_observations(const std::vector<common_lib::structures::Cone>& positions) override;
};
