
#include "kalman_filter/data_association.hpp"

#include <float.h>

#include <iostream>

DataAssociationModel::DataAssociationModel(float max_landmark_distance)
    : max_landmark_distance_(max_landmark_distance){};

float DataAssociationModel::get_max_landmark_distance() const {
  return this->max_landmark_distance_;
}

float SimpleMaximumLikelihood::curvature_ = 15.0;
float SimpleMaximumLikelihood::initial_limit_ = 0.1;

SimpleMaximumLikelihood::SimpleMaximumLikelihood(float max_landmark_distance)
    : DataAssociationModel(max_landmark_distance) {
  if (max_landmark_distance < 1) {
    throw std::invalid_argument("Invalid parameters for SimpleMaximumLikelihood");
  }
};

bool SimpleMaximumLikelihood::valid_match(const float delta,
                                          const float distance_to_vehicle) const {
  auto limit_function = [](float distance, float curv, float init_limit) {
    return pow(M_E, distance / curv) - (1 - init_limit);
  };
  return limit_function(distance_to_vehicle, SimpleMaximumLikelihood::curvature_,
                        SimpleMaximumLikelihood::initial_limit_) > delta;
}

int SimpleMaximumLikelihood::match_cone(const Eigen::Vector2f& observed_landmark_absolute,
                                        const Eigen::VectorXf& expected_state) const {
  // Check if landmark is within valid distance
  float distance_to_vehicle = (observed_landmark_absolute - expected_state.segment<2>(0)).norm();
  if (distance_to_vehicle > get_max_landmark_distance()) {
    return -1;
  }

  float min_delta = std::numeric_limits<float>::max();
  int closest_landmark_index = -1;
  for (int i = 5; i < expected_state.size(); i += 2) {
    float delta = (observed_landmark_absolute - expected_state.segment<2>(i)).norm();
    if (delta < min_delta) {
      min_delta = delta;
      closest_landmark_index = i;
    }
  }

  if (SimpleMaximumLikelihood::valid_match(min_delta, distance_to_vehicle)) {
    return closest_landmark_index;
  }

  return -2;
}