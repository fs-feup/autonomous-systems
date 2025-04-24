#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

NearestNeighbor::NearestNeighbor(const DataAssociationParameters& params)
    : DataAssociationModel(params) {}

Eigen::VectorXi NearestNeighbor::associate(const Eigen::VectorXd& landmarks,
                                           const Eigen::VectorXd& observations,
                                           const Eigen::MatrixXd& covariance,
                                           const Eigen::VectorXd& observation_confidences) const {
  const int num_observations = observations.size() / 2;
  const int num_landmarks = landmarks.size() / 2;

  Eigen::VectorXi associations =
      Eigen::VectorXi::Constant(num_observations, -2);  // Default: not associated (-2)

  for (int i = 0; i < num_observations; ++i) {
    Eigen::Vector2d obs;
    obs << observations(2 * i),
        observations(2 * i + 1);  // TODO: make another one with relative coordinates or a flag

    double best_cost = std::numeric_limits<double>::max();
    int best_landmark_index = -1;

    for (int j = 0; j < num_landmarks; j++) {
      const double euclidean_distance =
          std::hypot(obs(0) - landmarks(2 * j), obs(1) - landmarks(2 * j + 1));

      // Check if this landmark is a good candidate for association
      if (euclidean_distance < best_cost) {
        best_cost = euclidean_distance;
        best_landmark_index = 2 * j;
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
