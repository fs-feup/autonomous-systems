#include "kalman_filter/observation_models.hpp"

// TODO(marhcouto): add colors

Eigen::Vector2f ObservationModel::inverse_observation_model(
    const Eigen::VectorXf& expected_state, const ObservationData& observation_data) const {
  const double cone_x = observation_data.position.x + expected_state(0);
  const double cone_y = observation_data.position.y + expected_state(1);

  return Eigen::Vector2f(cone_x, cone_y);
}

Eigen::Vector2f ObservationModel::observation_model(const Eigen::VectorXf& expected_state,
                                                    const unsigned int landmark_index) const {
  const double delta_x = expected_state(landmark_index) - expected_state(0);
  const double delta_y = expected_state(landmark_index + 1) - expected_state(1);

  return Eigen::Vector2f(delta_x, delta_y);  // TODO(marhcouto): fix color
}

Eigen::MatrixXf ObservationModel::get_jacobian(const unsigned int landmark_index,
                                               const unsigned int state_size) const {
  Eigen::MatrixXf validation_jacobian = Eigen::MatrixXf::Zero(2, state_size);
  validation_jacobian(0, 0) = -1;
  validation_jacobian(1, 1) = -1;
  validation_jacobian(0, landmark_index) = 1;
  validation_jacobian(1, landmark_index + 1) = 1;

  return validation_jacobian;
}