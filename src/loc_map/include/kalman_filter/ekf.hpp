#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_

#include <Eigen/Dense>

#include "loc_map/data_structures.hpp"

// Eigen::Vector3d motion_model(const Eigen::Vector3d& localization, const Eigen::) //

/**
 * @brief Extended Kalman Filter class
 *
 * @details
 * The Extended Kalman Filter (EKF) is a recursive state estimator for nonlinear systems.
 * It is a nonlinear version of the Kalman Filter (KF).
 * This implementation is used to performe state estimation for the localization of
 * the vehicle and the map.
 *
 *
 *
 */
class ExtendedKalmanFilter {
  Eigen::MatrixXd A; /**< Recursive state transition matrix: describes how the state evolves from
                        one time step to the next */
  Eigen::MatrixXd
      B; /**< Motion to state matrix: describes how the controls/odometry affect the state */
  Eigen::MatrixXd C; /**< Measuremenets to state matrix: describes how the sensor measurements
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

  void next_state(Eigen::VectorXd& state, Eigen::VectorXd& control, Eigen::VectorXd& measurement);
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_