
#include "kalman_filter/data_association.hpp"

#include <float.h>

#include <fstream>
#include <iostream>

DataAssociationModel::DataAssociationModel(float max_landmark_distance)

    : max_landmark_distance_(max_landmark_distance){};

float DataAssociationModel::get_max_landmark_distance() const {
  return this->max_landmark_distance_;
}

float MaxLikelihood::nis_gate_ = 4.991f;  // Default value, can be overridden
float MaxLikelihood::nd_gate_ = 20.0f;
MaxLikelihood::MaxLikelihood(float max_landmark_distance)
    : DataAssociationModel(max_landmark_distance) {
  if (max_landmark_distance < 1) {
    throw std::invalid_argument("Invalid parameters for MaxLikelihood");
  }
};

int MaxLikelihood::associate_n_filter(
    const std::vector<common_lib::structures::Cone>& perception_map, Eigen::VectorXf& _x_vector_,
    Eigen::MatrixXf& _p_matrix_, std::vector<int>& matched_ids,
    std::vector<Eigen::Vector2f>& matched_cone_positions,
    std::vector<Eigen::Vector2f>& new_features, ObservationModel* observation_model) const {
  for (const common_lib::structures::Cone& cone : perception_map) {
    Eigen::Vector2f landmark_absolute = observation_model->inverse_observation_model(
        _x_vector_, ObservationData(cone.position.x, cone.position.y,
                                    common_lib::competition_logic::Color::BLUE));

    float distance_to_vehicle = (landmark_absolute - _x_vector_.segment<2>(0)).norm();
    if (distance_to_vehicle > this->get_max_landmark_distance()) {
      continue;
    }
    int j_best = 0;
    float n_best = std::numeric_limits<int>::max();
    float outer = std::numeric_limits<int>::max();
    for (int j = 6; j < _x_vector_.size(); j += 2) {
      // get expect observation from the state vector with the observation model
      Eigen::Vector2f z_hat = observation_model->observation_model(_x_vector_, j);

      Eigen::MatrixXf h_matrix = observation_model->get_state_to_observation_matrix(
          _x_vector_, j, static_cast<unsigned int>(_x_vector_.size()));
      // get the observation noise covariance matrix
      Eigen::MatrixXf q_matrix = observation_model->get_observation_noise_covariance_matrix();

      Eigen::Vector2f innovation = Eigen::Vector2f(cone.position.x, cone.position.y) - z_hat;

      Eigen::MatrixXf s_matrix = h_matrix * _p_matrix_ * h_matrix.transpose() + q_matrix;
      // calculate nis, that means normalized innovation squared
      float nis = innovation.transpose() * s_matrix.inverse() * innovation;

      float nd = nis + log(s_matrix.determinant());
      if (nis < this->nis_gate_ && nd < n_best) {  // 4.991
        n_best = nd;
        j_best = j;
      } else if (nis < outer) {
        outer = nis;  // outer is the closest unmatched distanced
      }
    }
    if (j_best != 0) {
      matched_ids.push_back(j_best);
      matched_cone_positions.push_back(Eigen::Vector2f(cone.position.x, cone.position.y));
    } else if (outer > this->nd_gate_) {  // 20 TUNE
      new_features.push_back(Eigen::Vector2f(cone.position.x, cone.position.y));
    }
  }
  return 0;
};
