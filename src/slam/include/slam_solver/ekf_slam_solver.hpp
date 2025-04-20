#pragma once

#include <Eigen/Dense>
#include <chrono>

#include "common_lib/conversions/cones.hpp"
#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/observation_model/base_observation_model.hpp"
#include "slam_solver/slam_solver.hpp"

class EKFSLAMSolver : public SLAMSolver {
  SLAMParameters slam_parameters_;
  std::shared_ptr<ObservationModel> observation_model_;
  Eigen::VectorXd state_ = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd process_noise_matrix_;
  Eigen::VectorXd pose = Eigen::VectorXd::Zero(3);

  rclcpp::Time last_update_;

  bool velocities_received_ = false;
  bool cones_receieved_ = false;

  /**
   * @brief Get the observation noise matrix object used in the correction step of the EKF
   * with the right dimensions
   *
   * @param num_landmarks number of observed landmarks in a given EKF correction step
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd get_observation_noise_matrix(int num_landmarks) const;

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
  void predict(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
               const Eigen::MatrixXd& process_noise_matrix, const rclcpp::Time last_update,
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
  void correct(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
               const std::vector<int>& observed_landmarks_indices,
               const Eigen::VectorXd& matched_observations);

  /**
   * @brief add new landmarks to the state vector and covariance matrix
   *
   * @details Expands the covariance matrix which originally has the structure:
   * P = [ Pcc, Pcl
   *       Plc, Pll]
   * To include the new landmarks, the covariance matrix is expanded to:
   * P = [ Pcc, Pcl, Pcn
   *       Plc, Pll, Pln
   *       Pnc, Pnl, Pnn]
   * consider C to be the car's position and orientation, L to be the landmarks that were already in
   * the state vector and N to be the new landmarks to be added to the state vector:
   * Pcc is the covariance of C with C, Pcl is the covariance of C with L, Plc is the covariance of
   * L with C, ...
   *
   * The new landmarks are added to the state vector after being converted to the global frame.
   *
   * @param state state vector with the car's position and orientation, followed by the landmarks in
   * the form {car_x, car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param covariance covariance matrix of the state vector
   * @param new_landmarks new landmarks in the form {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...} in
   * the car's frame
   */
  void state_augmentation(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
                          const Eigen::VectorXd& new_landmarks);

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
                std::shared_ptr<V2PMotionModel> motion_model,
                std::shared_ptr<LandmarkFilter> landmark_filter,
                std::shared_ptr<std::vector<double>> execution_times,
                std::weak_ptr<rclcpp::Node> node);
  /**
   * @brief Executed to deal with new velocity data
   *
   * @param velocities
   */
  void add_motion_prior(const common_lib::structures::Velocities& velocities) override;

  /**
   * @brief process obervations of landmarks
   *
   * @param position
   */
  void add_observations(const std::vector<common_lib::structures::Cone>& positions) override;

  /**
   * @brief Get the covariance matrix of the EKF
   *
   * @return Eigen::MatrixXd covariance matrix
   */
  Eigen::MatrixXd get_covariance() override { return covariance_; }
};
