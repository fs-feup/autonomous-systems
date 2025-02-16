#pragma once

#include <Eigen/Dense>
#include <chrono>

#include "common_lib/maths/transformations.hpp"
#include "slam_solver/slam_solver.hpp"

class EKFSLAMSolver : public SLAMSolver {
  SLAMParameters slam_parameters_;
  Eigen::VectorXd state_ = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd covariance_ = Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd process_noise_matrix_;

  std::chrono::high_resolution_clock::time_point last_update_;

  Eigen::VectorXd observed_landmarks_;

  bool velocities_received_ = false;
  bool cones_receieved_ = false;

public:
  EKFSLAMSolver(const SLAMParameters& params) : SLAMSolver(params), slam_parameters_(params) {
    this->process_noise_matrix_ = Eigen::MatrixXd::Zero(3, 3);
    this->process_noise_matrix_(0, 0) = params.velocity_x_noise_;
    this->process_noise_matrix_(1, 1) = params.velocity_y_noise_;
    this->process_noise_matrix_(2, 2) = params.angular_velocity_noise_;
  }

  /**
   * @brief Get the observation noise matrix object used in the correction step of the EKF
   * with the right dimensions
   *
   * @param num_landmarks number of observed landmarks in a given EKF correction step
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd get_observation_noise_matrix(size_t num_landmarks) const;

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
  void add_observations(const std::vector<common_lib::structures::Position>& positions);

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
               const Eigen::MatrixXd& process_noise_matrix,
               const std::chrono::high_resolution_clock::time_point last_update,
               const common_lib::structures::Velocities& velocities);

  /**
   * @brief predict observation based on the current state
   *
   * @param state vector with the car's position and orientation, followed by the landmark positions
   * {car_x, car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param matched_landmarks indexes of the x coordinate of landmarks in the state vector that were
   * observed in a specific measurement
   * @return Eigen::VectorXd predicted observations {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   */
  Eigen::VectorXd observations_prediction(const Eigen::VectorXd& state,
                                          const std::vector<int> matched_landmarks);

  /**
   * @brief inverse of the observation prediction: transforms the observations from the car's frame
   * to the global frame
   *
   * @param state used to get the car's pose
   * @param observations observed landmarks in the car's frame in form {x_cone_1, y_cone_1,
   * x_cone_2, y_cone_2, ...}
   * @return Eigen::VectorXd landmarks in the global frame in form {x_cone_1, y_cone_1, x_cone_2,
   * y_cone_2, ...}
   */
  Eigen::VectorXd inverse_of_observations_prediction(const Eigen::VectorXd& state,
                                                     const Eigen::VectorXd& observations);

  /**
   * @brief jacobian of the observation prediction
   *
   * @param state vector with the car's position and orientation, followed by the landmark positions
   * {car_x, car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param matched_landmarks indexes of the x coordinate of landmarks in the state vector that were
   * observed in a specific measurement
   * @return Eigen::MatrixXd jacobian of the observation prediction
   */
  Eigen::MatrixXd observations_prediction_jacobian(const Eigen::VectorXd& state,
                                                   const std::vector<int> matched_landmarks);

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

  /**
   * @brief get the Gv matrix used in the state augmentation function, calculated as the jacobian of
   * the inverse observation model with respect to the car's position and orientation
   *
   * @param state car's position and orientation, followed by the landmarks in the form {car_x,
   * car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param new_landmarks new landmarks in the form {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @return Eigen::MatrixXd Gv matrix of size (2 * num_new_landmarks, 3)
   */
  Eigen::MatrixXd gv(const Eigen::VectorXd& state, const Eigen::VectorXf& new_landmarks);

  /**
   * @brief get the Gz matrix used in the state augmentation function, calculated as the jacobian of
   * the inverse observation model with respect to the new landmarks positions (in the car's frame)
   *
   * @param state car's position and orientation, followed by the landmarks in the form {car_x,
   * car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param new_landmarks new landmarks in the form {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @return Eigen::MatrixXd Gz matrix of size (2 * num_new_landmarks, 2 * num_new_landmarks)
   */
  Eigen::MatrixXd gz(const Eigen::VectorXd& state, const Eigen::VectorXf& new_landmarks);
};