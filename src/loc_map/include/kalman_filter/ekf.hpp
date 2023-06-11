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
Eigen::VectorXf motion_model_expected_state(const Eigen::VectorXf& expected_state,
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
 * @return Eigen::MatrixXf
 */
Eigen::MatrixXf motion_model_covariance_matrix(const Eigen::VectorXf& state_covariance_matrix,
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
  Eigen::VectorXf X; /**< Expected state vector (localization + mapping) */
  Eigen::MatrixXf P; /**< State covariance  matrix */
  Eigen::MatrixXf R; /**< Motion noise covariance matrix */
  Eigen::MatrixXf Q; /**< Measurement noise covariance matrix */

  VehicleState* _vehicle_state; /**< Pointer to the vehicle state to be published */
  Map* _map;                    /**< Pointer to the map to be published */
  ImuUpdate* _imu_update;       /**< Pointer to the IMU update */

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

  void next_state(const Eigen::VectorXf& control,
                  const Eigen::VectorXf& measurement);  // Not implemented yet

  /**
   * @brief Updates vehicle state and map variables according 
   * to the state vector X
   * 
   */
  void update();

  /**
   * @brief Prediction step:
   * 1. Calculate the expected state regarding pose estimates
   * 2. Calculate the expected state covariance matrix regarding pose estimates
  */
  void prediction_step();

  VehicleState* get_vehicle_state() const { return this->_vehicle_state; }
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_