#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_

#include <Eigen/Dense>

#include "kalman_filter/motion_models.hpp"
#include "loc_map/data_structures.hpp"


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
  Eigen::MatrixXf P; /**< State covariance matrix */
  Eigen::MatrixXf R; /**< Motion noise matrix */
  Eigen::MatrixXf Q; /**< Measurement noise matrix */

  VehicleState* _vehicle_state; /**< Pointer to the vehicle state to be published */
  Map* _map;                    /**< Pointer to the map to be published */
  ImuUpdate* _imu_update;       /**< Pointer to the IMU update */
  Map* _map_from_perception;    /**< Pointer to the map predicted from perception */
  std::chrono::time_point<std::chrono::high_resolution_clock>
      _last_update; /**< Timestamp of last update */

  const MotionModel& _motion_model; /**< Motion Model chosen for prediction step */

 public:
  /**
   * @brief Construct a new Extended Kalman Filter object
   *
   * @param vehicle_state
   * @param map
   * @param imu_update data retrieved by the IMU
   */
  ExtendedKalmanFilter(Eigen::MatrixXf R, Eigen::MatrixXf Q, VehicleState* vehicle_state, Map* map,
                       ImuUpdate* imu_update, Map* map_from_perception,
                       const MotionModel& motion_model);

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

  /**
   * @brief Correction step:
   * 1. Calculate the Kalman Gain
   * 2. Calculate the expected measurement
   * 3. Calculate the expected measurement covariance matrix
   *
   */
  void correction_step();

  VehicleState* get_vehicle_state() const { return this->_vehicle_state; }
  Map* get_map() const { return this->_map; }
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_