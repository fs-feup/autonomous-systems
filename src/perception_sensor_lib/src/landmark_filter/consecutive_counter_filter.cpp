#include "perception_sensor_lib/landmark_filter/consecutive_counter_filter.hpp"

#include <map>

#include "common_lib/maths/transformations.hpp"
Eigen::VectorXd ConsecutiveCounterFilter::filter(const Eigen::VectorXd& observations,
                                                 const Eigen::VectorXd& observation_confidences,
                                                 Eigen::VectorXi& associations) {
  if (this->_params_.minimum_observation_count_ <= 1) {
    return observations;  // No filtering needed
  }

  std::vector<int>
      filter_to_global_observation_map;  // TODO: change to more efficient data structure

  Eigen::VectorXd unfiltered_new_observations;
  Eigen::VectorXd unfiltered_new_observations_confidences;
  for (int i = 0; i < associations.size(); i++) {
    if (associations(i) < 0) {
      unfiltered_new_observations.conservativeResize(unfiltered_new_observations.size() + 2);
      unfiltered_new_observations(unfiltered_new_observations.size() - 2) = observations(i * 2);
      unfiltered_new_observations(unfiltered_new_observations.size() - 1) = observations(i * 2 + 1);
      unfiltered_new_observations_confidences.conservativeResize(
          unfiltered_new_observations_confidences.size() + 1);
      unfiltered_new_observations_confidences(unfiltered_new_observations_confidences.size() - 1) =
          observation_confidences(i);
      associations(i) = -2;  // Mark as not ready to be added to the graph
      filter_to_global_observation_map.push_back(i);
    }
  }

  // Associate the observations that are considered new with the map stored in this filter
  Eigen::VectorXi new_associations = this->_data_association_->associate(
      this->map, unfiltered_new_observations, {}, unfiltered_new_observations_confidences);
  const int num_landmarks = this->map.size() / 2;
  const int num_new_observations = unfiltered_new_observations.size() / 2;

  // Calculate which observations should be added to this filter's map and which landmarks in the
  // map were observed
  std::vector<int> was_observed_as(num_landmarks, -1);  // Map landmark index to observation index
  Eigen::VectorXd to_be_added_to_map;
  for (int observation_index = 0; observation_index < num_new_observations; observation_index++) {
    if (new_associations(observation_index) != -1 && new_associations(observation_index) != -2) {
      was_observed_as[new_associations(observation_index) / 2] =
          observation_index;  // Map landmark index to observation index
    } else if (new_associations(observation_index) == -1) {
      to_be_added_to_map.conservativeResize(to_be_added_to_map.size() + 2);
      to_be_added_to_map.segment(to_be_added_to_map.size() - 2, 2) =
          unfiltered_new_observations.segment(observation_index * 2, 2);
    }
  }
  // Update counter and set filtered observations
  Eigen::VectorXd new_map;
  Eigen::VectorXi new_counter;
  Eigen::VectorXd filtered_observations;
  for (int landmark_index = 0; landmark_index < num_landmarks; landmark_index++) {
    if (was_observed_as[landmark_index] < 0) {
      continue;
    }
    if (this->counter(landmark_index) >= this->_params_.minimum_observation_count_ - 1) {
      filtered_observations.conservativeResize(filtered_observations.size() + 2);
      filtered_observations.segment(filtered_observations.size() - 2, 2) =
          unfiltered_new_observations.segment(was_observed_as[landmark_index] * 2, 2);
      associations(filter_to_global_observation_map[was_observed_as[landmark_index]]) =
          -1;  // Mark as ready to be added to the graph
    }
    new_map.conservativeResize(new_map.size() + 2);
    new_map.segment(new_map.size() - 2, 2) = this->map.segment(landmark_index * 2, 2);
    new_counter.conservativeResize(new_counter.size() + 1);
    new_counter(new_counter.size() - 1) = this->counter(landmark_index) + 1;
  }
  new_map.conservativeResize(new_map.size() + to_be_added_to_map.size());
  new_map.tail(to_be_added_to_map.size()) = to_be_added_to_map;
  new_counter.conservativeResize(new_counter.size() + to_be_added_to_map.size() / 2);
  new_counter.tail(to_be_added_to_map.size() / 2).setOnes();
  this->map = new_map;
  this->counter = new_counter;

  return filtered_observations;
}

void ConsecutiveCounterFilter::delete_landmarks(const Eigen::VectorXd& some_landmarks) {
  int map_size = this->map.size() / 2;
  int num_landmarks = some_landmarks.size() / 2;
  std::vector<bool> to_be_deleted(map_size, false);
  Eigen::VectorXd new_map;
  Eigen::VectorXi new_counter;
  for (int i = 0; i < num_landmarks; i++) {
    for (int j = 0; j < map_size; j++) {
      if (std::hypot(this->map(j * 2) - some_landmarks(i * 2),
                     this->map(j * 2 + 1) - some_landmarks(i * 2 + 1)) < EQUALITY_TOLERANCE) {
        to_be_deleted[j] = true;
      }
    }
  }
  for (int i = 0; i < map_size; i++) {
    if (to_be_deleted[i]) {
      continue;
    }
    new_map.conservativeResize(new_map.size() + 2);
    new_map.segment(new_map.size() - 2, 2) = this->map.segment(i * 2, 2);
    new_counter.conservativeResize(new_counter.size() + 1);
    new_counter(new_counter.size() - 1) = this->counter(i);
  }
  this->map = new_map;
  this->counter = new_counter;
}