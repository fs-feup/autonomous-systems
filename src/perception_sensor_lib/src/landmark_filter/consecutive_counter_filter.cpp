#include "perception_sensor_lib/landmark_filter/consecutive_counter_filter.hpp"

#include <iostream>
Eigen::VectorXd ConsecutiveCounterFilter::filter(const Eigen::VectorXd& state,
                                                 const Eigen::MatrixXd& covariance,
                                                 const Eigen::VectorXd& observations,
                                                 const Eigen::VectorXd& observation_confidences,
                                                 const Eigen::VectorXi& associations) {
  // Get only the observations that are considered new relative to the SLAM map
  Eigen::VectorXd new_observations;
  Eigen::VectorXd new_confidences;
  int number_of_observations = associations.size();
  for (int i = 0; i < number_of_observations; i++) {
    if (associations(i) == -1) {
      new_observations.conservativeResize(new_observations.size() + 2);
      new_observations.segment(new_observations.size() - 2, 2) = observations.segment(i * 2, 2);
      new_confidences.conservativeResize(new_confidences.size() + 1);
      new_confidences(new_confidences.size() - 1) = observation_confidences(i);
    }
  }
  // Associate the observations that are considered new with the map stored in this filter
  this->map.segment(0, 3) = state.segment(0, 3);
  Eigen::VectorXi new_associations = this->_data_association_->associate(
      this->map, Eigen::MatrixXd::Zero(this->map.size(), this->map.size()), new_observations,
      new_confidences);

  // Calculate which observations should be added to this filer's map and which landmarks in the map
  // were observed
  std::vector<bool> was_observed((this->map.size() - 3) / 2, false);
  Eigen::VectorXd to_be_added_to_map;
  for (int i = 0; i < new_associations.size(); i++) {
    if (new_associations(i) != -1 && new_associations(i) != -2) {
      was_observed[(new_associations(i) - 3) / 2] = true;
    } else if (new_associations(i) == -1) {
      to_be_added_to_map.conservativeResize(to_be_added_to_map.size() + 2);
      to_be_added_to_map.segment(to_be_added_to_map.size() - 2, 2) =
          new_observations.segment(i * 2, 2);
    }
  }
  // Update counter
  Eigen::VectorXd new_map = Eigen::VectorXd::Zero(3);
  Eigen::VectorXi new_counter;
  Eigen::VectorXd filtered_observations;
  for (int i = 0; i < was_observed.size(); i++) {
    if (!was_observed[i]) {
      continue;
    } else {
      if (this->counter(i) == this->_params_.minimum_number_of_observations - 1) {
        filtered_observations.conservativeResize(filtered_observations.size() + 2);
        filtered_observations.segment(filtered_observations.size() - 2, 2) =
            this->map.segment(i * 2 + 3, 2);
      } else {
        new_map.conservativeResize(new_map.size() + 2);
        new_map.segment(new_map.size() - 2, 2) = this->map.segment(i * 2 + 3, 2);
        new_counter.conservativeResize(new_counter.size() + 1);
        new_counter(new_counter.size() - 1) = this->counter(i) + 1;
      }
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