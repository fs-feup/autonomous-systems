#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_

#include <Eigen/Dense>

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
  virtual Eigen::VectorXf predict_expected_state(
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
  virtual Eigen::MatrixXf get_motion_to_state_matrix(const Eigen::VectorXf& expected_state,
                                              const MotionPredictionData& motion_prediction_data,
                                              const double time_interval) const = 0;
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
  Eigen::VectorXf predict_expected_state(const Eigen::VectorXf& expected_state,
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
  Eigen::MatrixXf get_motion_to_state_matrix(const Eigen::VectorXf& expected_state,
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
  Eigen::VectorXf predict_expected_state(const Eigen::VectorXf& expected_state,
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
  Eigen::MatrixXf get_motion_to_state_matrix(const Eigen::VectorXf& expected_state,
                                              const MotionPredictionData& motion_prediction_data,
                                              const double time_interval) const override;
};


#endif // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_