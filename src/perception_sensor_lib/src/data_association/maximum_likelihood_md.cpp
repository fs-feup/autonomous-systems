#include "perception_sensor_lib/data_association/maximum_likelihood_md.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

std::vector<int> MaximumLikelihoodMD::associate(
    const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
    const Eigen::VectorXd& observations, const Eigen::VectorXd& observation_confidences) const {
  int num_observations = observations.size() / 2;
  std::vector<int> associations(num_observations, -2);  // Default: not associated (-2)

  double car_x = state(0);
  double car_y = state(1);
  double car_theta = state(2);
  double cos_theta = std::cos(car_theta);
  double sin_theta = std::sin(car_theta);

  // Generate indexes of all landmarks in state vector
  std::vector<int> vec(num_observations);
  for (int i = 0; i < num_observations; ++i) {
    vec[i] = 2 * i + 3;
  }

  // Get coordinates of all landmarks in local frame
  Eigen::VectorXd landmarks_local_coordinates = common_lib::maths::global_to_local_referential(
      state.segment(0, 3), state.segment(3, state.size() - 3));

  // Set fields of the jacobian that are common to all landmarks
  Eigen::MatrixXd jacobian_local_coordinates = Eigen::MatrixXd::Zero(2, state.size());
  jacobian_local_coordinates(0, 0) = -cos_theta;
  jacobian_local_coordinates(0, 1) = -sin_theta;
  jacobian_local_coordinates(1, 0) = sin_theta;
  jacobian_local_coordinates(1, 1) = -cos_theta;

  for (int i = 0; i < num_observations; ++i) {
    Eigen::Vector2d obs;
    obs << observations(2 * i), observations(2 * i + 1);

    double best_cost = std::numeric_limits<double>::max();
    int best_landmark_index = -1;

    for (int j = 3; j < state.size(); j += 2) {
      // Extract landmark global coordinates
      double lx = state(j);
      double ly = state(j + 1);
      double dx = lx - car_x;
      double dy = ly - car_y;

      // Set fields of the jacobian that are not common to all landmarks.
      jacobian_local_coordinates(0, 2) = -(-sin_theta * dx + cos_theta * dy);
      jacobian_local_coordinates(1, 2) = -(cos_theta * dx + sin_theta * dy);
      jacobian_local_coordinates(0, j) = cos_theta;
      jacobian_local_coordinates(0, j + 1) = sin_theta;
      jacobian_local_coordinates(1, j) = -sin_theta;
      jacobian_local_coordinates(1, j + 1) = cos_theta;

      // Check if the landmark is within a valid distance from the car
      double distance =
          std::hypot(landmarks_local_coordinates(j - 3), landmarks_local_coordinates(j - 2));
      if (distance > this->params_.max_landmark_distance) continue;

      // Innovation: difference between actual and predicted observation
      Eigen::Vector2d innovation = obs - landmarks_local_coordinates.segment(j - 3, 2);

      // Innovation covariance: S = H * covariance * H^T + Q
      Eigen::Matrix2d innovation_covariance =
          jacobian_local_coordinates * covariance * jacobian_local_coordinates.transpose() +
          this->observation_noise_covariance_matrix_;

      // Guard against non-invertible innovation_covariance
      double det = innovation_covariance.determinant();
      if (det == 0) continue;

      double normalized_innovation_squared =
          innovation.transpose() * innovation_covariance.inverse() * innovation;

      // Check if this landmark is a good candidate for association
      if (normalized_innovation_squared < best_cost) {
        best_cost = normalized_innovation_squared;
        best_landmark_index = j;
      }

      // Reset fields of the jacobian that are not common to all landmarks
      jacobian_local_coordinates(0, 2) = 0;
      jacobian_local_coordinates(1, 2) = 0;
      jacobian_local_coordinates(0, j) = 0;
      jacobian_local_coordinates(0, j + 1) = 0;
      jacobian_local_coordinates(1, j) = 0;
      jacobian_local_coordinates(1, j + 1) = 0;
    }

    if (best_cost < this->params_.association_gate) {
      associations[i] = best_landmark_index;
    } else if (observation_confidences[i] > this->params_.new_landmark_confidence_gate) {
      associations[i] = -1;  // New landmark
    }
  }

  return associations;
}