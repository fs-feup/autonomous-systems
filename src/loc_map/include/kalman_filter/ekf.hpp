#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_

#include <Eigen/Dense>
#include <vector>

#include "kalman_filter/motion_models.hpp"
#include "kalman_filter/observation_models.hpp"
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
  static double max_landmark_distance; /**< Maximum deviation of the landmark position from the
                                          expected position when the landmark is perceived
                                          to be 1 meter away */

  Eigen::VectorXf X;                   /**< Expected state vector (localization + mapping) */
  Eigen::MatrixXf P;                   /**< State covariance matrix */
  std::vector<colors::Color> _colors;  /**< Vector of colors of the landmarks */

  std::chrono::time_point<std::chrono::high_resolution_clock>
      _last_update;                           /**< Timestamp of last update */

  const MotionModel& _motion_model;           /**< Motion Model chosen for prediction step */
  const ObservationModel& _observation_model; /**< Observation Model chosen for correction step */

  MotionUpdate _last_motion_update;           /**< Last motion update */

  bool _fixed_map = false;                    /**< Flag to indicate if the map is fixed */

  /**
   * @brief Discovery step:
   * Adds a new landmark to the map
   * if there are no matches for the observation
   *
   * @param observation_data
   * @return int index of the landmark or, in case of rejection, -1
   */
  int discovery(const ObservationData& observation_data);

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

  /**
   * @brief Check if the cone matches with the landmark
   * the score is greater the best the match is
   *
   * @param x_from_state x absolute coordinate of the landmark in state
   * @param y_from_state y absolute coordinate of the landmark in state
   * @param x_from_perception x coordinate of the landmark from perception
   * @param y_from_perception y coordinate of the landmark from perception
   * @param distance_to_vehicle distance of the cone to the vehicle
   * @return difference between limit and delta (score)
   */
  static bool cone_match(const double x_from_state, const double y_from_state,
                         const double x_from_perception, const double y_from_perception,
                         const double distance_to_vehicle);

 public:
  /**
   * @brief Construct a new Extended Kalman Filter object
   *
   * @param motion_model motion model chosen for prediction step
   * @param observation_model observation model chosen for correction step
   */
  ExtendedKalmanFilter(const MotionModel& motion_model, const ObservationModel& observation_model);

  /**
   * @brief Build the EKF for specific events
   *
   * @param ekf
   * @return ExtendedKalmanFilter
   */
  ExtendedKalmanFilter(const MotionModel& motion_model, const ObservationModel& observation_model,
                       const Mission& mission);

  /**
   * @brief Updates vehicle state and map variables according
   * to the state vector X
   *
   * @param vehicle_state pose
   * @param track_map map
   *
   */
  void update(VehicleState* vehicle_state, ConeMap* track_map);

  /**
   * @brief Prediction step:
   * 1. Calculate the expected state regarding pose estimates
   * 2. Calculate the expected state covariance matrix regarding pose estimates
   *
   * @param motion_update data of motion (velocities)
   */
  void prediction_step(const MotionUpdate& motion_update);

  /**
   * @brief Correction step:
   * 1. Calculate the Kalman Gain
   * 2. Calculate the expected measurement
   * 3. Calculate the expected measurement covariance matrix
   *
   * @param perception_map map from perception
   */
  void correction_step(const ConeMap& perception_map);

  Eigen::VectorXf get_state() const { return this->X; }

  Eigen::MatrixXf get_covariance() const { return this->P; }

  std::chrono::time_point<std::chrono::high_resolution_clock> get_last_update() const {
    return this->_last_update;
  }
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_EKF_HPP_