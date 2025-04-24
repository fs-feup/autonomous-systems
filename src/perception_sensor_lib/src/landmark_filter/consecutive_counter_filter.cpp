#include "perception_sensor_lib/landmark_filter/consecutive_counter_filter.hpp"

#include <iostream>

#include "common_lib/maths/transformations.hpp"
Eigen::VectorXd ConsecutiveCounterFilter::filter(
    const Eigen::VectorXd& new_observations, const Eigen::VectorXd& new_observation_confidences) {
  // Associate the observations that are considered new with the map stored in this filter
  Eigen::VectorXi new_associations = this->_data_association_->associate(
      this->map, new_observations, {}, new_observation_confidences);
  const int num_landmarks = this->map.size() / 2;
  const int num_new_observations = new_observations.size() / 2;

  // Calculate which observations should be added to this filter's map and which landmarks in the
  // map were observed
  std::vector<bool> was_observed(num_landmarks, false);
  Eigen::VectorXd to_be_added_to_map;
  for (int i = 0; i < num_new_observations; i++) {
    if (new_associations(i) != -1 && new_associations(i) != -2) {
      was_observed[new_associations(i) / 2] = true;
    } else if (new_associations(i) == -1) {
      to_be_added_to_map.conservativeResize(to_be_added_to_map.size() + 2);
      to_be_added_to_map.segment(to_be_added_to_map.size() - 2, 2) =
          new_observations.segment(i * 2, 2);
    }
  }
  // Update counter
  Eigen::VectorXd new_map = Eigen::VectorXd::Zero(0);
  Eigen::VectorXi new_counter;
  Eigen::VectorXd filtered_observations;
  for (int i = 0; i < num_landmarks; i++) {
    if (!was_observed[i]) {
      continue;
    }
    if (this->counter(i) >= this->_params_.minimum_observation_count_ - 1) {
      filtered_observations.conservativeResize(filtered_observations.size() + 2);
      for (int j = 0; j < num_new_observations; j++) {
        if (new_associations[j] == i * 2) {
          filtered_observations.segment(filtered_observations.size() - 2, 2) =
              new_observations.segment(j * 2, 2);
          break;
        }
      }
      filtered_observations.segment(filtered_observations.size() - 2, 2) =
          this->map.segment(i * 2, 2);
    } else {
      new_map.conservativeResize(new_map.size() + 2);
      new_map.segment(new_map.size() - 2, 2) = this->map.segment(i * 2, 2);
      new_counter.conservativeResize(new_counter.size() + 1);
      new_counter(new_counter.size() - 1) = this->counter(i) + 1;
    }
  }
  new_map.conservativeResize(new_map.size() + to_be_added_to_map.size());
  new_map.tail(to_be_added_to_map.size()) = to_be_added_to_map;
  new_counter.conservativeResize(new_counter.size() + to_be_added_to_map.size() / 2);
  new_counter.tail(to_be_added_to_map.size() / 2).setOnes();
  this->map = new_map;
  this->counter = new_counter;

  return filtered_observations;
}