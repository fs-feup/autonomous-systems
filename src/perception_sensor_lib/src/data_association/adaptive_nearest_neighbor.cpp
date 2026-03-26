#include "perception_sensor_lib/data_association/adaptive_nearest_neighbor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

AdaptiveNearestNeighbor::AdaptiveNearestNeighbor(const DataAssociationParameters& params)
    : DataAssociationModel(params) {}

void AdaptiveNearestNeighbor::get_tentative_nearest_neighbors(
    const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
    const Eigen::VectorXd& observation_confidences, const Eigen::Vector3d& pose,
    Eigen::VectorXi& nearest_landmarks, Eigen::VectorXd& nearest_distances) const {
  const int num_observations = observations.size() / 2;
  const int num_landmarks = landmarks.size() / 2;

  nearest_landmarks = Eigen::VectorXi::Constant(num_observations, -1);
  nearest_distances =
      Eigen::VectorXd::Constant(num_observations, std::numeric_limits<double>::infinity());

  for (int observation_idx = 0; observation_idx < num_observations; ++observation_idx) {
    if (observation_confidences(observation_idx) < this->_params_.new_landmark_confidence_gate) {
      continue;
    }

    if (const double observation_distance_to_pose =
            std::hypot(observations(2 * observation_idx) - pose(0),
                       observations(2 * observation_idx + 1) - pose(1));
        observation_distance_to_pose > this->_params_.seed_radius) {
      continue;
    }

    for (int landmark_idx = 0; landmark_idx < num_landmarks; ++landmark_idx) {
      const double distance =
          std::hypot(observations(2 * observation_idx) - landmarks(2 * landmark_idx),
                     observations(2 * observation_idx + 1) - landmarks(2 * landmark_idx + 1));

      if (distance < nearest_distances(observation_idx)) {
        nearest_distances(observation_idx) = distance;
        nearest_landmarks(observation_idx) = landmark_idx;
      }
    }
  }
}

Eigen::VectorXi AdaptiveNearestNeighbor::get_seed_observation_indices(
    const Eigen::VectorXi& nearest_landmarks, const Eigen::VectorXd& nearest_distances) const {
  const int max_seed_observations = std::max(2, this->_params_.seed_count);
  const int num_seed_slots =
      std::min(static_cast<int>(nearest_landmarks.size()), max_seed_observations);
  Eigen::VectorXi closest_observation_indices = Eigen::VectorXi::Constant(num_seed_slots, -1);
  Eigen::VectorXd closest_distances =
      Eigen::VectorXd::Constant(num_seed_slots, std::numeric_limits<double>::infinity());

  for (int observation_idx = 0; observation_idx < nearest_landmarks.size(); ++observation_idx) {
    if (nearest_landmarks(observation_idx) == -1 ||
        !std::isfinite(nearest_distances(observation_idx))) {
      continue;
    }

    for (int insert_idx = 0; insert_idx < num_seed_slots; ++insert_idx) {
      if (nearest_distances(observation_idx) >= closest_distances(insert_idx)) {
        continue;
      }

      for (int shift_idx = num_seed_slots - 1; shift_idx > insert_idx; --shift_idx) {
        closest_distances(shift_idx) = closest_distances(shift_idx - 1);
        closest_observation_indices(shift_idx) = closest_observation_indices(shift_idx - 1);
      }

      closest_distances(insert_idx) = nearest_distances(observation_idx);
      closest_observation_indices(insert_idx) = observation_idx;
      break;
    }
  }

  int num_seed_observations = 0;
  while (num_seed_observations < closest_observation_indices.size() &&
         closest_observation_indices(num_seed_observations) != -1) {
    ++num_seed_observations;
  }

  Eigen::VectorXi seed_observation_indices(num_seed_observations);
  for (int seed_idx = 0; seed_idx < num_seed_observations; ++seed_idx) {
    seed_observation_indices(seed_idx) = closest_observation_indices(seed_idx);
  }

  return seed_observation_indices;
}

bool AdaptiveNearestNeighbor::estimate_transform_from_pair(
    const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
    int first_observation_idx, int first_landmark_idx, int second_observation_idx,
    int second_landmark_idx, Eigen::Vector3d& transform) const {
  if (first_observation_idx == second_observation_idx ||
      first_landmark_idx == second_landmark_idx) {
    return false;
  }

  const Eigen::Vector2d first_observation = observations.segment(2 * first_observation_idx, 2);
  const Eigen::Vector2d second_observation = observations.segment(2 * second_observation_idx, 2);
  const Eigen::Vector2d first_landmark = landmarks.segment(first_landmark_idx, 2);
  const Eigen::Vector2d second_landmark = landmarks.segment(second_landmark_idx, 2);

  const Eigen::Vector2d observation_delta = second_observation - first_observation;
  const Eigen::Vector2d landmark_delta = second_landmark - first_landmark;

  const double rotation =
      common_lib::maths::normalize_angle(std::atan2(landmark_delta.y(), landmark_delta.x()) -
                                         std::atan2(observation_delta.y(), observation_delta.x()));
  const Eigen::Matrix2d rotation_matrix = common_lib::maths::get_rotation_matrix(rotation);
  const Eigen::Vector2d translation = first_landmark - rotation_matrix * first_observation;

  transform << translation.x(), translation.y(), rotation;
  return true;
}

void AdaptiveNearestNeighbor::score_associations(const Eigen::VectorXd& landmarks,
                                                 const Eigen::VectorXd& observations,
                                                 const Eigen::VectorXi& associations,
                                                 int& num_associations,
                                                 double& total_distance) const {
  num_associations = 0;
  total_distance = 0.0;

  for (int observation_idx = 0; observation_idx < associations.size(); ++observation_idx) {
    if (associations(observation_idx) < 0) {
      continue;
    }

    ++num_associations;
    total_distance +=
        std::hypot(observations(2 * observation_idx) - landmarks(associations(observation_idx)),
                   observations(2 * observation_idx + 1) -
                       landmarks(associations(observation_idx) + 1));
  }
}

Eigen::VectorXi AdaptiveNearestNeighbor::associate(const Eigen::VectorXd& landmarks,
                                                    const Eigen::VectorXd& observations,
                                                    const Eigen::MatrixXd& covariance,
                                                    const Eigen::VectorXd& observation_confidences,
                                                    const Eigen::Vector3d& pose) const {
  NearestNeighbor nearest_neighbor(this->_params_);

  Eigen::VectorXi best_associations =
      nearest_neighbor.associate(landmarks, observations, covariance, observation_confidences, pose);
  int best_num_associations = 0;
  double best_total_distance = 0.0;
  score_associations(landmarks, observations, best_associations, best_num_associations,
                     best_total_distance);

  Eigen::VectorXi nearest_landmarks;
  Eigen::VectorXd nearest_distances;
  get_tentative_nearest_neighbors(landmarks, observations, observation_confidences, pose,
                                  nearest_landmarks, nearest_distances);

  const Eigen::VectorXi seed_observation_indices =
      get_seed_observation_indices(nearest_landmarks, nearest_distances);

  auto try_pair = [&](const int first_seed_idx, const int second_seed_idx) {
    Eigen::Vector3d transform;
    if (!estimate_transform_from_pair(
            landmarks, observations, seed_observation_indices(first_seed_idx),
            2 * nearest_landmarks(seed_observation_indices(first_seed_idx)),
            seed_observation_indices(second_seed_idx),
            2 * nearest_landmarks(seed_observation_indices(second_seed_idx)), transform)) {
      return;
    }

    const Eigen::VectorXd adjusted_observations = adjust_observations(observations, transform);
    const Eigen::VectorXi associations = nearest_neighbor.associate(
        landmarks, adjusted_observations, covariance, observation_confidences, pose);

    int num_associations = 0;
    double total_distance = 0.0;
    score_associations(landmarks, adjusted_observations, associations, num_associations,
                       total_distance);

    if (num_associations > best_num_associations ||
        (num_associations == best_num_associations && total_distance < best_total_distance)) {
      best_associations = associations;
      best_num_associations = num_associations;
      best_total_distance = total_distance;
    }
  };

  for (int first_seed_idx = 0; first_seed_idx < seed_observation_indices.size(); ++first_seed_idx) {
    for (int second_seed_idx = first_seed_idx + 1;
         second_seed_idx < seed_observation_indices.size(); ++second_seed_idx) {
      try_pair(first_seed_idx, second_seed_idx);
    }
  }

  return best_associations;
}

Eigen::VectorXd AdaptiveNearestNeighbor::adjust_observations(
    const Eigen::VectorXd& observations, const Eigen::Vector3d& transform) const {
  Eigen::VectorXd adjusted_observations(observations.size());
  const Eigen::Matrix2d rotation_matrix = common_lib::maths::get_rotation_matrix(transform(2));

  for (int point_idx = 0; point_idx < observations.size(); point_idx += 2) {
    adjusted_observations.segment(point_idx, 2) =
        rotation_matrix * observations.segment(point_idx, 2) + transform.head<2>();
  }

  return adjusted_observations;
}
