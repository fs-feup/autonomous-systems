#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_

#include <Eigen/Dense>

#include "loc_map/data_structures.hpp"

/**
 * @brief Struct containing motion prediction data
 * Depending on the motion model, some data may be irrelevant
 *
 */
struct MotionPredictionData {
  double translational_velocity = 0.0;   /**< Translational Velocity Mod */
  double translational_velocity_x = 0.0; /**< Translational Velocity in X axis */
  double translational_velocity_y = 0.0; /**< Translational Velocity in Y axis */
  double rotational_velocity = 0.0;      /**< Rotational Velocity */
};

/**
 * @brief Abstract Moiton Model class
 * designed to be implemented by the different motion models
 *
 */
class MotionModel {
 public:
  /**
   * @brief Calculate expected state vector from
   * motion estimation
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::VectorXf
   */
  virtual Eigen::VectorXf motion_model_expected_state(
      const Eigen::VectorXf& expected_state, const MotionPredictionData& motion_prediction_data,
      const double time_interval) const = 0;

  /**
   * @brief Calculate state covariance matrix from
   * motion estimation
   *
   * @param state_covariance_matrix
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  virtual Eigen::MatrixXf motion_model_covariance_matrix(
      const Eigen::MatrixXf& state_covariance_matrix, const Eigen::MatrixXf& motion_noise_matrix,
      const MotionPredictionData& motion_prediction_data, const double time_interval) const = 0;
};

/**
 * @brief Motion estimation that uses
 * the specific values of the x and y axis
 * accelerations, being the angle of the
 * vehicle and its position independent
 *
 */
class ImuVelocityModel : public MotionModel {
 public:
  /**
   * @brief Calculate expected state vector from
   * velocity model using IMU data and linear functions
   * Uses translational velocity in x and y axis
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::VectorXf
   */
  Eigen::VectorXf motion_model_expected_state(const Eigen::VectorXf& expected_state,
                                              const MotionPredictionData& motion_prediction_data,
                                              const double time_interval) const override;
  /**
   * @brief Calculate state covariance matrix from
   * velocity model using IMU data and linear functions
   * Uses translational velocity in x and y axis
   *
   * @param state_covariance_matrix
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf motion_model_covariance_matrix(const Eigen::MatrixXf& state_covariance_matrix,
                                                 const Eigen::MatrixXf& motion_noise_matrix,
                                                 const MotionPredictionData& motion_prediction_data,
                                                 const double time_interval) const override;
};

/**
 * @brief Motion estimation that uses
 * the module of the velocity and the
 * rotational velocity
 *
 */
class NormalVelocityModel : public MotionModel {
 public:
  /**
   * @brief Calculate expected state vector from
   * velocity model using normal motion data
   * Uses translation velocity only
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::VectorXf
   */
  Eigen::VectorXf motion_model_expected_state(const Eigen::VectorXf& expected_state,
                                              const MotionPredictionData& motion_prediction_data,
                                              const double time_interval) const override;

  /**
   * @brief Calculate state covariance matrix from
   * velocity model using normal motion data
   * Uses translation velocity only
   *
   * @param state_covariance_matrix
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf motion_model_covariance_matrix(const Eigen::MatrixXf& state_covariance_matrix,
                                                 const Eigen::MatrixXf& motion_noise_matrix,
                                                 const MotionPredictionData& motion_prediction_data,
                                                 const double time_interval) const override;
};

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