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
Eigen::VectorXd motion_model_expected_state(const Eigen::VectorXd& expected_state,
                                            const float translational_velocity_x,
                                            const float translational_velocity_y,
                                            const float rotational_velocity_z,
                                            const double time_interval);

/**
 * @brief Function to calculate the new state covariance matrix using the jacobian of the motion
 * model
 *
 * @param state_covariance_matrix
 * @param translational_velocity_x
 * @param translational_velocity_y
 * @param rotational_velocity_z
 * @param time_interval
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd motion_model_covariance_matrix(const Eigen::VectorXd& state_covariance_matrix,
                                               const float translational_velocity_x,
                                               const float translational_velocity_y,
                                               const float rotational_velocity_z,
                                               const double time_interval);

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
  Eigen::MatrixXd R; /**< Motion noise covariance matrix */
  Eigen::MatrixXd Q; /**< Measurement noise covariance matrix */
  Eigen::MatrixXd P; /**< State error covariance  matrix */
  Eigen::VectorXd X; /**< Expected state vector (localization + mapping) */

  ImuUpdate* _imu_update;       /**< Pointer to the IMU update */
  VehicleState* _vehicle_state; /**< Pointer to the vehicle state to be published */
  Map* _map;                    /**< Pointer to the map to be published */

 public:
  /**
   * @brief Construct a new Extended Kalman Filter object
   *
   * @param vehicle_state
   * @param map
   * @param imu_update data retrieved by the IMU
   */
  ExtendedKalmanFilter(
      VehicleState* vehicle_state, Map* map,
      ImuUpdate* imu_update);  // TODO(marhcouto): add constructor that uses noise matrixes

  void next_state(const Eigen::VectorXd& control,
                  const Eigen::VectorXd& measurement);  // Not implemented yet
  void prediction_step();
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_