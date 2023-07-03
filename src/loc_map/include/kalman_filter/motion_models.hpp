#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_

#include <Eigen/Dense>

/**
 * @brief Struct containing motion prediction data
 * Depending on the motion model, some data may be irrelevant
 *
 */
struct MotionPredictionData {
  // IMU
  double translational_velocity = 0.0;   /**< Translational Velocity Mod */
  double translational_velocity_x = 0.0; /**< Translational Velocity in X axis */
  double translational_velocity_y = 0.0; /**< Translational Velocity in Y axis */
  double rotational_velocity = 0.0;      /**< Rotational Velocity */

  // Odometry
  double steering_angle = 0.0; /**< Steering Angle */
  double lf_speed = 0.0;       /**< Left Front Wheel Speed */
  double rf_speed = 0.0;       /**< Right Front Wheel Speed */
  double lb_speed = 0.0;       /**< Left Back Wheel Speed */
  double rb_speed = 0.0;       /**< Right Back Wheel Speed */
};

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
   * @param process_noise_covariance_matrix covariance matrix of the process noise (R)
   */
  explicit MotionModel(const Eigen::MatrixXf& process_noise_covariance_matrix);

  /**
   * @brief Calculate expected state vector from
   * motion estimation
   *
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::VectorXf
   */
  virtual Eigen::VectorXf predict_expected_state(const Eigen::VectorXf& expected_state,
                                                 const MotionPredictionData& motion_prediction_data,
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
      const Eigen::VectorXf& expected_state, [[maybe_unused]] const MotionPredictionData& motion_prediction_data,
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
   * @param process_noise_covariance_matrix covariance matrix of the process noise (R)
   */
  explicit ImuVelocityModel(const Eigen::MatrixXf& process_noise_covariance_matrix);

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
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_motion_to_state_matrix(const Eigen::VectorXf& expected_state,
                                             [[maybe_unused]] const MotionPredictionData& motion_prediction_data,
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
   * @param process_noise_covariance_matrix covariance matrix of the process noise (R)
   */
  explicit NormalVelocityModel(const Eigen::MatrixXf& process_noise_covariance_matrix);

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
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_motion_to_state_matrix(const Eigen::VectorXf& expected_state,
                                             [[maybe_unused]] const MotionPredictionData& motion_prediction_data,
                                             [[maybe_unused]] const double time_interval) const override;
};

class OdometryModel : public NormalVelocityModel {
  Eigen::MatrixXf _process_noise_covariance_matrix;
  static constexpr double wheelbase = 1.530;    /**< space between axises in meters */
  static constexpr double axis_length = 1.2;    /**< space between wheels in meters */
  static constexpr double wheel_diameter = 0.5; /**< wheel radius in meters */

 public:
  /**
   * @brief Construct a new Motion Model object
   *
   * @param process_noise_covariance_matrix covariance matrix of the process noise (R)
   */
  explicit OdometryModel(const Eigen::MatrixXf& process_noise_covariance_matrix);

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
   * @param expected_state
   * @param motion_prediction_data
   * @param time_interval in seconds
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_motion_to_state_matrix(const Eigen::VectorXf& expected_state,
                                             [[maybe_unused]] const MotionPredictionData& motion_prediction_data,
                                             [[maybe_unused]] const double time_interval) const override;

  /**
   * @brief Get the wheel velocity in m/s from rpm
   * 
   * @param rpm rotations per minute
   * @return double 
   */
  static double get_wheel_velocity_from_rpm(const double rpm);

  /**
   * @brief Transform odometry data to velocities
   *  (wheel speeds and steering to linear and angular velocities)
   *
   * @param motion_prediction_data
   * @return MotionPredictionData
   */
  MotionPredictionData odometry_to_velocities_transform(
      const MotionPredictionData& motion_prediction_data) const;
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_MOTION_MODELS_HPP_