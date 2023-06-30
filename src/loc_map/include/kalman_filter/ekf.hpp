#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_

#include <Eigen/Dense>
#include <vector>

#include "kalman_filter/motion_models.hpp"
#include "kalman_filter/observation_models.hpp"
#include "loc_map/data_structures.hpp"

/**
 * @brief Notas
 *
 * - No inicio, software engineers specialized in AI - no
 * - Integration with buses of ifrastructure in system design - no
 * - We will not be developping simulation models - no
 * - Not vehicles from the bus manufacturing company - no
 *
 *
 */

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
  static float max_landmark_deviation; /**< Maximum deviation of the landmark position from the
                                          expected position */

  Eigen::VectorXf X;                  /**< Expected state vector (localization + mapping) */
  Eigen::MatrixXf P;                  /**< State covariance matrix */
  std::vector<colors::Color> _colors; /**< Vector of colors of the landmarks */

  VehicleState* _vehicle_state; /**< Pointer to the vehicle state to be published */
  Map* _map;                    /**< Pointer to the map to be published */
  ImuUpdate* _imu_update;       /**< Pointer to the IMU update */
  Map* _map_from_perception;    /**< Pointer to the map predicted from perception */
  std::chrono::time_point<std::chrono::high_resolution_clock>
      _last_update; /**< Timestamp of last update */

  const MotionModel& _motion_model;           /**< Motion Model chosen for prediction step */
  const ObservationModel& _observation_model; /**< Observation Model chosen for correction step */

  /**
   * @brief Discovery step:
   * Adds a new landmark to the map
   * if there are no matches for the observation
   *
   * @param observation_data
   * @return unsigned int
   */
  unsigned int discovery(const ObservationData& observation_data);

  /**
   * @brief Calculate the kalman gain
   *
   * @param H jacobian observation model matrix
   * @param P state covariance matrix
   * @param Q measurement noise matrix
   * @return Eigen::MatrixXf kalman gain matrix
   */
  Eigen::MatrixXf get_kalman_gain(const Eigen::MatrixXf& H, const Eigen::MatrixXf& P,
                                  const Eigen::MatrixXf& Q);

 public:
  /**
   * @brief Construct a new Extended Kalman Filter object
   *
   * @param vehicle_state pointer to the vehicle state to be published
   * @param map pointer to the map to be published
   * @param imu_update pointer to the data coming from IMU
   * @param map_from_perception pointer to the map predicted from perception
   * @param motion_model motion model chosen for prediction step
   * @param observation_model observation model chosen for correction step
   */
  ExtendedKalmanFilter(VehicleState* vehicle_state, Map* map, ImuUpdate* imu_update,
                       Map* map_from_perception, const MotionModel& motion_model,
                       const ObservationModel& observation_model);

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
  std::chrono::time_point<std::chrono::high_resolution_clock> get_last_update() const {
    return this->_last_update;
  }
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_