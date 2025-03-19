#include "perception_sensor_lib/data_association/maximum_likelihood_md.hpp"

#include <cmath>
#include <limits>
#include <vector>

Eigen::VectorXi MaximumLikelihoodMD::associate(
    const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
    const Eigen::VectorXd& observations, const Eigen::VectorXd& observation_confidences) const {
  int num_observations = observations.size() / 2;
  Eigen::VectorXi associations =
      Eigen::VectorXi::Constant(num_observations, -2);  // Default: not associated (-2)

  double car_x = state(0);
  double car_y = state(1);
  double car_theta = state(2);
  double cos_theta = std::cos(car_theta);
  double sin_theta = std::sin(car_theta);

  // Get coordinates of all landmarks in local frame
  Eigen::VectorXd landmarks_local_coordinates = common_lib::maths::global_to_local_coordinates(
      state.segment(0, 3), state.segment(3, state.size() - 3));

  for (int i = 0; i < num_observations; ++i) {
    Eigen::Vector2d obs;
    obs << observations(2 * i), observations(2 * i + 1);

    double best_cost = std::numeric_limits<double>::max();
    int best_landmark_index = -1;

    for (int j = 3; j < state.size(); j += 2) {
      // Check if the landmark is within a valid distance from the car
      double distance =
          std::hypot(landmarks_local_coordinates(j - 3), landmarks_local_coordinates(j - 2));
      if (distance > this->_params_.max_landmark_distance) continue;

      // Extract landmark global coordinates
      double lx = state(j);
      double ly = state(j + 1);
      double dx = lx - car_x;
      double dy = ly - car_y;

      // Innovation: difference between actual and predicted observation
      Eigen::Vector2d innovation = obs - landmarks_local_coordinates.segment(j - 3, 2);

      Eigen::Matrix2d innovation_covariance = covariance.block(j, j, 2, 2);
      // Guard against non-invertible innovation_covariance
      double det = innovation_covariance.determinant();
      if (det == 0) {
        RCLCPP_WARN(rclcpp::get_logger("data_association"),
                    "Non-invertible innovation covariance matrix");
        continue;
      }
      double normalized_innovation_squared =
          innovation.transpose() * innovation_covariance.inverse() * innovation;

      // Check if this landmark is a good candidate for association
      if (normalized_innovation_squared < best_cost) {
        best_cost = normalized_innovation_squared;
        best_landmark_index = j;
      }
    }

    if (best_cost < this->_params_.association_gate) {
      associations(i) = best_landmark_index;
    } else if (observation_confidences[i] > this->_params_.new_landmark_confidence_gate) {
      associations(i) = -1;  // New landmark
    }
  }

  return associations;
}