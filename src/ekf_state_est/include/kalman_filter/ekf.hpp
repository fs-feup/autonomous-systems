#pragma once

#include <gtest/gtest_prod.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/landmark.hpp"
#include "common_lib/structures/vehicle_state.hpp"
#include "kalman_filter/data_association.hpp"
#include "kalman_filter/motion_models.hpp"
#include "kalman_filter/observation_models.hpp"

/**
 * @brief Extended Kalman Filter class
 *
 * @details
 * The Extended Kalman Filter (EKF) is a recursive state estimator for nonlinear
 * systems. It is a nonlinear version of the Kalman Filter (KF). This
 * implementation is used to perform state estimation for the localization of
 * the vehicle and the map.
 *
 */
class ExtendedKalmanFilter {
  Eigen::VectorXf _x_vector_ =
      Eigen::VectorXf::Zero(6); /**< Expected state vector (localization + mapping) */
  Eigen::MatrixXf _p_matrix_ = Eigen::MatrixXf::Identity(6, 6);
  rclcpp::Time _last_update_ = rclcpp::Clock().now(); /**< Timestamp of last update */

  std::map<std::string, std::shared_ptr<MotionModel>> _motion_models_;  // Map of motion models
  std::shared_ptr<ObservationModel>
      _observation_model_; /**< Observation Model chosen for correction step */
  std::shared_ptr<DataAssociationModel> _data_association_model_; /**< Data Association Model*/
  std::vector<common_lib::structures::Landmark>
      _landmarksDatabase_; /**< Vector of landmarks used for mapping */

  bool _fixed_map = false;         /**< Flag to indicate if the map is fixed */
  bool _first_prediction_ = true;  /// Flags used to mark first prediction step

  /**
   * @brief Calculate the kalman gain
   *
   * @param H jacobian observation model matrix
   * @param _p_matrix_ state covariance matrix
   * @param Q measurement noise matrix
   * @return Eigen::MatrixXf kalman gain matrix
   */
  Eigen::MatrixXf get_kalman_gain(const Eigen::MatrixXf &H, const Eigen::MatrixXf &_p_matrix_,
                                  const Eigen::MatrixXf &Q) const;

public:
  /**
   * @brief Extended Kalman Filter constructor declaration
   *
   * @param motion_model motion model chosen for prediction step
   * @param observation_model observation model chosen for correction step
   */
  ExtendedKalmanFilter(std::shared_ptr<ObservationModel> observation_model,
                       std::shared_ptr<DataAssociationModel> data_association_model);

  /**
   * @brief Updates vehicle state and map variables according
   * to the state vector _x_vector_
   *
   * @param vehicle_state pose
   * @param track_map map
   */
  void update(std::shared_ptr<common_lib::structures::VehicleState> vehicle_state,
              std::shared_ptr<std::vector<common_lib::structures::Cone>> track_map);

  /**
   * @brief Prediction step:
   * 1. Calculate the expected state regarding pose estimates
   * 2. Calculate the expected state covariance matrix regarding pose estimates
   *
   * @param motion_update data of motion (velocities)
   */
  void prediction_step(const MotionUpdate &motion_update, const std::string &sensor_type);

  void add_motion_model(const std::string &model_name, std::shared_ptr<MotionModel> motion_model) {
    _motion_models_[model_name] = motion_model;
  }

  /**
   * @brief Correction step:
   * 1. Calculate the Kalman Gain
   * 2. Calculate the expected measurement
   * 3. Calculate the expected measurement covariance matrix
   *
   * @param perception_map map from perception
   */
  void correction_step(const std::vector<common_lib::structures::Cone> &perception_map);
  //   void wss_correction_step(const MotionUpdate &motion_correction_data);
  /**
   * @brief Get the state vector
   *
   * @return Eigen::VectorXf state vector
   */
  Eigen::VectorXf get_state() const { return this->_x_vector_; }

  /**
   * @brief Get the state covariance matrix
   *
   * @return Eigen::MatrixXf covariance matrix
   */
  Eigen::MatrixXf get_covariance() const { return this->_p_matrix_; }

  /**
   * @brief Get the last update timestamp
   *
   * @return rclcpp::Time
   * timestamp
   */
  rclcpp::Time get_last_update() const { return this->_last_update_; }

  friend class PerformanceTest;
};