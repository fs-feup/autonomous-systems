
#include "kalman_filter/data_association.hpp"

#include <float.h>

#include <fstream>
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

bool SimpleMaximumLikelihood::validate(const Eigen::Vector2f& predicted_measurement,
                                       const Eigen::Vector2f& observed_measurement,
                                       const Eigen::MatrixXf& covariance, int landmark_index,
                                       const Eigen::MatrixXf& R, const Eigen::MatrixXf& H) const {
  // Get the expected landmark from the state
  if (landmark_index < 0) {
    return false;
  }
  std::ofstream file("matrices.txt", std::ios::app);
  // file << "H" << H << "\n";
  // file << "covariance" << covariance << "\n";
  file.flush();
  // Eigen::Matrix2f P = covariance.block<2, 2>(landmark_index, landmark_index);
  // file << "P" << P << "\n";
  file.flush();
  // Eigen::Matrix2f H_landmark = H.block<2, 2>(0, landmark_index);
  // file << "H" << H_landmark << "\n";
  file.flush();
  Eigen::Vector2f v = predicted_measurement - observed_measurement;
  // print matrices to file

  // file << "P" << P << "\n";
  // file << "V" << v << "\n";
  // file << "R" << R << "\n";

  file.flush();

  // Eigen::Matrix2f R_block = R.block<2, 2>(0, 0);

  Eigen::Matrix2f S = H * covariance * H.transpose() + R /* R_block */;
  float validation_value = v.transpose() * S.inverse() * v;
  // output to ////file valçue and v values
  // file << "VALIDATION VALUE" << validation_value << "\n";
  // file << "V" << v << "\n";
  file.flush();
  S.diagonal().array() += 1e-6;
  float nd = validation_value + log(S.determinant());
  return nd < 7.815;
}
int SimpleMaximumLikelihood::match_cone(const Eigen::Vector2f& observed_landmark_absolute,
                                        const Eigen::VectorXf& expected_state) const {
  // Check if landmark is within valid distance
  float distance_to_vehicle = (observed_landmark_absolute - expected_state.segment<2>(0)).norm();
  if (distance_to_vehicle > get_max_landmark_distance()) {
    return -1;
  }

  float min_delta = std::numeric_limits<float>::max();
  int closest_landmark_index = -2;
  for (int i = 6; i < expected_state.size(); i += 2) {
    float delta = (observed_landmark_absolute - expected_state.segment<2>(i)).norm();
    if (delta < min_delta) {
      min_delta = delta;
      closest_landmark_index = i;
    }
  }

  return closest_landmark_index;
}