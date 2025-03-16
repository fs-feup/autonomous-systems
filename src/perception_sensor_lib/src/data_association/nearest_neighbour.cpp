#include "perception_sensor_lib/data_association/nearest_neighbour.hpp"

Eigen::VectorXi NearestNeighbour::associate(const Eigen::VectorXd& state,
                                            const Eigen::MatrixXd& covariance,
                                            const Eigen::VectorXd& observations,
                                            const Eigen::VectorXd& observation_confidences) const {
  // Transform observations to global coordinates
  Eigen::VectorXd global_observations =
      common_lib::maths::local_to_global_coordinates(state.segment(0, 3), observations);

  double squared_distance_threshold =
      this->_params_.association_gate * this->_params_.association_gate;
  double new_landmark_squared_distance_gate = 1;

  Eigen::VectorXi associations = Eigen::VectorXi::Constant(observations.size() / 2, -2);

  std::unordered_set<int> used_indices((state.size() - 3) / 2);

  for (int i = 0; i < observations.size(); i += 2) {
    if (observations(i) * observations(i) + observations(i + 1) * observations(i + 1) >
        this->_params_.max_landmark_distance) {
      continue;
    }

    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for (int j = 3; j < state.size(); j += 2) {
      double squared_distance =
          (state(j) - global_observations(i)) * (state(j) - global_observations(i)) +
          (state(j + 1) - global_observations(i + 1)) * (state(j + 1) - global_observations(i + 1));

      if (squared_distance < min_distance) {
        min_distance = squared_distance;
        min_index = j;
      }
    }

    if (observation_confidences(i / 2) >= this->_params_.new_landmark_confidence_gate ||
        used_indices.find(min_index) == used_indices.end()) {
      if (min_distance < squared_distance_threshold) {
        associations(i / 2) = min_index;
        used_indices.insert(min_index);
      } else if (min_distance > new_landmark_squared_distance_gate) {
        associations(i / 2) = -1;
      }
    }
  }
  return associations;
}