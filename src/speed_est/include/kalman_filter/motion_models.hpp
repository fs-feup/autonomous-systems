#ifndef SRC_SPEED_EST_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_
#define SRC_SPEED_EST_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_

#include <eigen3/Eigen/Dense>

#include "speed_est/data_structures.hpp"

/**
 * @brief Abstract Moiton Model class
 * designed to be implemented by the different motion models
 *
 */
class MotionModel {
  Eigen::MatrixXf _process_noise_covariance_matrix; /**< R */

 public:
  /**
   * @brief Construct a new Motion Model object
   *
   * @param process_noise_covariance_matrix covariance matrix of the process
   * noise (R)
   */
  explicit MotionModel(const Eigen::MatrixXf &process_noise_covariance_matrix);

  /**
   * @brief Calculate expected state vector from
   * motion estimation
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::VectorXf
   */
  virtual Eigen::VectorXf predict_expected_state(const Eigen::VectorXf &expected_state,
                                                 const MotionUpdate &motion_prediction_data,
                                                 const double time_interval) const = 0;

  /**
   * @brief Calculate state covariance matrix from
   * motion estimation
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  virtual Eigen::MatrixXf get_motion_to_state_matrix(
      const Eigen::VectorXf &expected_state,
      [[maybe_unused]] const MotionUpdate &motion_prediction_data,
      const double time_interval) const = 0;

  /**
   * @brief Get the process noise covariance matrix object
   *
   * @param state_size
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_process_noise_covariance_matrix(const unsigned int state_size) const;
};

/**
 * @brief Motion estimation that uses
 * the specific values of the x and y axis
 * accelerations, being the angle of the
 * vehicle and its position independent
 *
 */
class ImuVelocityModel : public MotionModel {
  Eigen::MatrixXf _process_noise_covariance_matrix;

 public:
  /**
   * @brief Construct a new Motion Model object
   *
   * @param process_noise_covariance_matrix covariance matrix of the process
   * noise (R)
   */
  explicit ImuVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix);

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
  Eigen::VectorXf predict_expected_state(const Eigen::VectorXf &expected_state,
                                         const MotionUpdate &motion_prediction_data,
                                         const double time_interval) const override;
  /**
   * @brief Calculate state covariance matrix from
   * velocity model using IMU data and linear functions
   * Uses translational velocity in x and y axis
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_motion_to_state_matrix(
      const Eigen::VectorXf &expected_state,
      [[maybe_unused]] const MotionUpdate &motion_prediction_data,
      [[maybe_unused]] const double time_interval) const override;
};

/**
 * @brief Motion estimation that uses
 * the module of the velocity and the
 * rotational velocity
 *
 */
class NormalVelocityModel : public MotionModel {
  Eigen::MatrixXf _process_noise_covariance_matrix;

 public:
  /**
   * @brief Construct a new Motion Model object
   *
   * @param process_noise_covariance_matrix covariance matrix of the process
   * noise (R)
   */
  explicit NormalVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix);

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
  Eigen::VectorXf predict_expected_state(const Eigen::VectorXf &expected_state,
                                         const MotionUpdate &motion_prediction_data,
                                         const double time_interval) const override;

  /**
   * @brief Calculate state covariance matrix from
   * velocity model using normal motion data
   * Uses translation velocity only
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_motion_to_state_matrix(
      const Eigen::VectorXf &expected_state,
      [[maybe_unused]] const MotionUpdate &motion_prediction_data,
      [[maybe_unused]] const double time_interval) const override;
};

#endif  // SRC_SPEED_EST_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_