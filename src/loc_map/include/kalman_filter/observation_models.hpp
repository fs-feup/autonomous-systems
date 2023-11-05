#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_CORRECTION_MODELS_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_CORRECTION_MODELS_HPP_

#include <Eigen/Dense>

#include "loc_map/data_structures.hpp"

/**
 * @brief Struct containing observation data
 *
 * @param position
 * @param color
 */
struct ObservationData {
  Position position;
  colors::Color color;
  ObservationData() : position(0, 0), color(colors::blue) {}
  ObservationData(Position position, colors::Color color) : position(position), color(color) {}
  ObservationData(double x, double y, colors::Color color) : position(x, y), color(color) {}
};

/**
 * @brief Observation Model class
 * compiled of functions for observation model
 */
class ObservationModel {
  Eigen::Matrix2f _observation_noise_covariance_matrix; /**< H or C */

 public:
  /**
   * @brief Construct a new Observation Model object
   *
   * @param observation_noise_covariance_matrix covariance matrix of the observation noise (Q)
   */
  explicit ObservationModel(const Eigen::Matrix2f& observation_noise_covariance_matrix);

  /**
   * @brief Calculate expected observation from
   * the state vector
   *
   * @param expected_state
   * @param landmark_index index of the x variable of the landmark in the state vector
   * @return Eigen::Vector2f
   */
  Eigen::Vector2f observation_model(const Eigen::VectorXf& expected_state,
                                    const unsigned int landmark_index) const;

  /**
   * @brief Calculate landmark position from
   * observation
   *
   *
   * @param expected_state
   * @param observation_data
   * @return Eigen::Vector2f
   */
  Eigen::Vector2f inverse_observation_model(const Eigen::VectorXf& expected_state,
                                            const ObservationData& observation_data) const;

  /**
   * @brief Get the state to observation matrix
   * of the observation model (H)
   *
   * @param landmark_index index of the x variable of the landmark in the state vector
   * @param state_size size of the state vector
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_state_to_observation_matrix(
      const Eigen::VectorXf& expected_state, const unsigned int landmark_index,
      const unsigned int state_size) const;  // TODO(marhcouto): refactor this maybe

  /**
   * @brief Get the observation noise covariance matrix (C or H)
   *
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_observation_noise_covariance_matrix() const;
};

#endif  // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_CORRECTION_MODELS_HPP_