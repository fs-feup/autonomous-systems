#include "perception_sensor_lib/data_association/nearest_neighbour.hpp"

Eigen::VectorXi NearestNeighbour::associate(const Eigen::VectorXd& landmarks,
                                            const Eigen::VectorXd& observations,
                                            const Eigen::MatrixXd& covariance,
                                            const Eigen::VectorXd& observation_confidences) const {
  double new_landmark_distance_gate = 1.2;
  const int num_observations = observations.size() / 2;
  const int num_landmarks = landmarks.size() / 2;

  Eigen::VectorXi associations = Eigen::VectorXi::Constant(observations.size() / 2, -2);

  std::unordered_set<int> used_indices(num_landmarks);

  for (int i = 0; i < num_observations; i++) {
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for (int j = 0; j < num_landmarks; j++) {
      const double euclidean_distance = std::hypot(observations(2 * i) - landmarks(2 * j),
                                                   observations(2 * i + 1) - landmarks(2 * j + 1));
      if (euclidean_distance < min_distance) {
        min_distance = euclidean_distance;
        min_index = 2 * j;
      }
    }

    if (observation_confidences(i) >= this->_params_.new_landmark_confidence_gate &&
        used_indices.find(min_index) == used_indices.end()) {
      if (min_distance < this->_params_.association_gate) {
        associations(i) = min_index;
        used_indices.insert(min_index);
      } else if (min_distance > new_landmark_distance_gate) {
        associations(i) = -1;
      }
    }
  }
  return associations;
}