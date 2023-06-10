#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_

#include <Eigen/Dense>

#include "loc_map/data_structures.hpp"

/**
 * @brief Function to calculate the next state of the vehicle
 * 
 * @param pose Pose of the vehicle 
 * @param translational_velocity_x meters per second
 * @param translational_velocity_y meters per second
 * @param rotational_velocity_z degrees per second
 * @param time_interval time interval in seconds
 * @return Eigen::Vector3d 
 */
Eigen::VectorXd motion_model_expected_state(const Eigen::VectorXd& expected_state, const float translational_velocity_x, const float translational_velocity_y, const float rotational_velocity_z, const double time_interval);
Eigen::MatrixXd motion_model_covariance_matrix(const Eigen::VectorXd& state_covariance_matrix, const float translational_velocity_x, const float translational_velocity_y, const float rotational_velocity_z, const double time_interval);

/**
 * @brief Extended Kalman Filter class
 *
 * @details
 * The Extended Kalman Filter (EKF) is a recursive state estimator for nonlinear systems.
 * It is a nonlinear version of the Kalman Filter (KF).
 * This implementation is used to performe state estimation for the localization of
 * the vehicle and the map.
 *
 */
class ExtendedKalmanFilter {
  Eigen::MatrixXd A; /**< Recursive state transition matrix: describes how the state evolves from
                        one time step to the next */
  Eigen::MatrixXd
      B; /**< Motion to state matrix: describes how the controls/odometry affect the state */
  Eigen::MatrixXd C; /**< Measurements to state matrix: describes how the sensor measurements
                        affect the state */
  Eigen::MatrixXd R; /**< Motion noise covariance matrix */
  Eigen::MatrixXd Q; /**< Measurement noise covariance matrix */
  Eigen::MatrixXd P; /**< State error covariance  matrix */
  Eigen::VectorXd expected_state; /**< Expected state vector (localization + mapping) */

 public:
  /**
   * @brief Construct a new Extended Kalman Filter object
   *
   * @param A - Recursive state transition matrix: describes how the state evolves from one time
   * step to the next
   * @param B - Motion to state matrix: describes how the controls/odometry affect the state
   * @param C - Measuremenets to state matrix: describes how the sensor measurements affect the
   * state
   * @param R - Motion noise matrix
   * @param Q - Measurement noise matrix
   */
  ExtendedKalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C,
                       const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q);

  void next_state(Eigen::VectorXd& control, Eigen::VectorXd& measurement);

  void prediction_step(Eigen::VectorXd& control, Eigen::VectorXd (*motion_model_function));
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_