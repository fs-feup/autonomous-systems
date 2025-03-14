#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

NearestNeighbor::NearestNeighbor(const DataAssociationParameters& params)
    : DataAssociationModel(params) {}

Eigen::VectorXi NearestNeighbor::associate(const Eigen::VectorXd& state,
                                           const Eigen::MatrixXd& covariance,
                                           const Eigen::VectorXd& observations,
                                           const Eigen::VectorXd& observation_confidences) const {
  const int num_observations = observations.size() / 2;

  Eigen::VectorXi associations =
      Eigen::VectorXi::Constant(num_observations, -2);  // Default: not associated (-2)

  // Get coordinates of all landmarks in local frame
  Eigen::VectorXd landmarks_local_coordinates = common_lib::maths::global_to_local_coordinates(
      state.segment(0, 3), state.segment(3, state.size() - 3));
  Eigen::VectorXd landmarks_global_coordinates = state.segment(3, state.size() - 3);
  Eigen::VectorXd obvservation_global_coordinates =
      common_lib::maths::local_to_global_coordinates(state.segment(0, 3), observations);

  for (int i = 0; i < num_observations; ++i) {
    Eigen::Vector2d obs;
    obs << obvservation_global_coordinates(2 * i), obvservation_global_coordinates(2 * i + 1);

    double best_cost = std::numeric_limits<double>::max();
    int best_landmark_index = -1;

    for (int j = 3; j < state.size(); j += 2) {
      // Check if the landmark is within a valid distance from the car
      if (const double distance =
              std::hypot(landmarks_local_coordinates(j - 3), landmarks_local_coordinates(j - 2));
          distance > this->_params_.max_landmark_distance) {
        continue;
      }

      const double euclidian_difference = std::hypot(obs(0) - landmarks_global_coordinates(j - 3),
                                                     obs(1) - landmarks_global_coordinates(j - 2));

      // Check if this landmark is a good candidate for association
      if (euclidian_difference < best_cost) {
        best_cost = euclidian_difference;
        best_landmark_index = j;
      }
    }

    if (best_cost < this->_params_.association_gate) {
      associations(i) = best_landmark_index;
    } else if (observation_confidences[i] > this->_params_.new_landmark_confidence_gate) {
      associations(i) = -1;  // New landmark
    }
  }

  return associations;
}
