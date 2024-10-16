#pragma once

#include <eigen3/Eigen/Dense>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/structures/position.hpp"

/**
 * @brief Struct containing observation data
 *
 * @param position
 * @param color
 */
struct ObservationData {
  common_lib::structures::Position position;
  common_lib::competition_logic::Color color;
  ObservationData() : position(0, 0), color(common_lib::competition_logic::Color::BLUE) {}
  ObservationData(common_lib::structures::Position position,
                  common_lib::competition_logic::Color color)
      : position(position), color(color) {}
  ObservationData(double x, double y, common_lib::competition_logic::Color color)
      : position(x, y), color(color) {}
};

/**
 * @brief Observation Model class
 * compiled of functions for observation model
 */
class ObservationModel {
  Eigen::MatrixXf _observation_noise_covariance_matrix_; /**< H or C */

public:
  /**
   * @brief Construct a new Observation Model object
   *
   * @param observation_noise_covariance_matrix covariance matrix of the
   * observation noise (Q)
   */
  explicit ObservationModel(const Eigen::MatrixXf &observation_noise_covariance_matrix);

  /**
   * @brief Calculate expected observation from
   * the state vector
   *
   * @param expected_state
   * @param landmark_index index of the x variable of the landmark in the state
   * vector
   * @return Eigen::Vector2f
   */
  Eigen::Vector2f observation_model(const Eigen::VectorXf &expected_state,
                                    const unsigned int landmark_index) const;

  Eigen::VectorXf observation_model_n_landmarks(const Eigen::VectorXf &current_state,
                                                const std::vector<int> &matched_ids) const;

  Eigen::VectorXf format_observation(const std::vector<Eigen::Vector2f> &observations) const;

  Eigen::MatrixXf get_jacobian_of_observation_model(const Eigen::VectorXf &current_state,
                                                    const std::vector<int> &matched_ids) const;

  Eigen::MatrixXf get_full_observation_noise_covariance_matrix(const int observation_size) const;

  /**
   * @brief Calculate landmark position from
   * observation
   *
   *
   * @param expected_state
   * @param observation_data
   * @return Eigen::Vector2f
   */
  Eigen::Vector2f inverse_observation_model(const Eigen::VectorXf &expected_state,
                                            const ObservationData &observation_data) const;

  Eigen::MatrixXf get_gv(const Eigen::VectorXf &expected_state,
                         const ObservationData &observation_data) const;

  Eigen::MatrixXf get_gz(const Eigen::VectorXf &expected_state,
                         const ObservationData &observation_data) const;

  /**
   * @brief Get the state to observation matrix
   * of the observation model (H)
   *
   * @param landmark_index index of the x variable of the landmark in the state
   * vector
   * @param state_size size of the state vector
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_state_to_observation_matrix(
      const Eigen::VectorXf &expected_state, const unsigned int landmark_index,
      const unsigned int state_size) const;  // TODO(marhcouto): refactor this maybe

  /**
   * @brief Get the observation noise covariance matrix (C or H)
   *
   * @return Eigen::MatrixXf
   */
  Eigen::MatrixXf get_observation_noise_covariance_matrix() const;

  /**
   * @brief Create a Q matrix from a given noise value
   *
   * @return Eigen::Matrix2f
   */
  static Eigen::MatrixXf create_observation_noise_covariance_matrix(float noise_value);
};
