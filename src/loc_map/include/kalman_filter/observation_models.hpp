#ifndef SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_CORRECTION_MODELS_HPP_
#define SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_CORRECTION_MODELS_HPP_

#include <Eigen/Dense>

#include "loc_map/data_structures.hpp"

/**
 * @brief Struct containing observation data
 * 
 * @param position
 * @param color
 * 
 */
struct ObservationData {
  Position position;
  colors::Color color;
  ObservationData(Position position, colors::Color color) : position(position), color(color) {}
  ObservationData(double x, double y, colors::Color color) : position(x, y), color(color) {}
};

/**
 * @brief Observation Model class
 * compiled of functions for observation model
 * 
 */
class ObservationModel {
 public:  
  Eigen::Vector2f observation_model(const Eigen::VectorXf& expected_state, const unsigned int landmark_index) const;
  Eigen::Vector2f inverse_observation_model(const Eigen::VectorXf& expected_state, const ObservationData& observation_data) const;
  Eigen::MatrixXf get_jacobian(const unsigned int landmark_index, const unsigned int state_size) const; // TODO(marhcouto): refactor this maybe
};



#endif // SRC_LOC_MAP_INCLUDE_KALMAN_FILTER_CORRECTION_MODELS_HPP_