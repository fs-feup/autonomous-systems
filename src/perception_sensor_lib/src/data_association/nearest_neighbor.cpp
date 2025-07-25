#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

#include <queue>
#include <tuple>

NearestNeighbor::NearestNeighbor(const DataAssociationParameters& params)
    : DataAssociationModel(params) {}

struct TripleComparator {
  bool operator()(const std::tuple<double, unsigned int, unsigned int>& a,
                  const std::tuple<double, unsigned int, unsigned int>& b) const {
    return std::get<0>(a) > std::get<0>(b);
  }
};

Eigen::VectorXi NearestNeighbor::associate(const Eigen::VectorXd& landmarks,
                                           const Eigen::VectorXd& observations,
                                           const Eigen::MatrixXd& covariance,
                                           const Eigen::VectorXd& observation_confidences) const {
  const int num_observations = observations.size() / 2;
  const int num_landmarks = landmarks.size() / 2;

  Eigen::VectorXi associations =
      Eigen::VectorXi::Constant(num_observations, -1);  // Default: new landmark

  std::priority_queue<std::tuple<double, unsigned int, unsigned int>,
                      std::vector<std::tuple<double, unsigned int, unsigned int>>, TripleComparator>
      distances;

  double euclidean_distance;
  for (int i = 0; i < num_observations; ++i) {
    if (observation_confidences(i) < this->_params_.new_landmark_confidence_gate) {
      associations(i) = -2;
      continue;
    }
    for (int j = 0; j < num_landmarks; ++j) {
      euclidean_distance = std::hypot(observations(2 * i) - landmarks(2 * j),
                                      observations(2 * i + 1) - landmarks(2 * j + 1));
      if (euclidean_distance < this->_params_.association_gate) {
        distances.emplace(euclidean_distance, i, j);
      }
    }
  }

  std::set<unsigned int> associated_landmarks;
  std::tuple<double, unsigned int, unsigned int> current_tuple;
  while (!distances.empty()) {
    current_tuple = distances.top();
    distances.pop();

    if (associated_landmarks.find(std::get<2>(current_tuple)) == associated_landmarks.end() &&
        associations(std::get<1>(current_tuple)) == -1) {
      associations(std::get<1>(current_tuple)) = 2 * std::get<2>(current_tuple);
      associated_landmarks.insert(std::get<2>(current_tuple));
    }
  }

  return associations;
}
