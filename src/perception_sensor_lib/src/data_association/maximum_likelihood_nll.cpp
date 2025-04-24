#include "perception_sensor_lib/data_association/maximum_likelihood_nll.hpp"

#include <cmath>
#include <limits>
#include <vector>

Eigen::VectorXi MaximumLikelihoodNLL::associate(
    const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
    const Eigen::MatrixXd& covariance, const Eigen::VectorXd& observation_confidences) const {
  const int num_observations = observations.size() / 2;
  const int num_landmarks = landmarks.size() / 2;
  Eigen::VectorXi associations =
      Eigen::VectorXi::Constant(num_observations, -2);  // Default: not associated (-2)

  for (int i = 0; i < num_observations; i++) {
    Eigen::Vector2d obs;
    obs << observations(2 * i), observations(2 * i + 1);

    double best_cost = std::numeric_limits<double>::max();
    int best_landmark_index = -1;

    for (int j = 0; j < num_landmarks; j++) {
      // Innovation: difference between observations and landmarks
      Eigen::Vector2d innovation = obs - landmarks.segment(2 * j, 2);

      Eigen::Matrix2d innovation_covariance =
          covariance.block(2 * j, 2 * j, 2, 2) + this->observation_noise_covariance_matrix_;

      // Guard against non-invertible innovation_covariance
      double det = innovation_covariance.determinant();
      if (det == 0) {
        RCLCPP_WARN(rclcpp::get_logger("data_association"),
                    "Non-invertible innovation covariance matrix for landmark at: [%f, %f]",
                    landmarks(2 * j), landmarks(2 * j + 1));
        continue;
      }
      if (det < 1e-9) det = 1e-9;

      double normalized_innovation_squared =
          innovation.transpose() * innovation_covariance.inverse() * innovation;
      double cost = normalized_innovation_squared + std::log(det);
      // Check if this landmark is a good candidate for association
      if (normalized_innovation_squared < this->_params_.association_gate && cost < best_cost) {
        best_cost = cost;
        best_landmark_index = 2 * j;
      }
    }

    if (best_landmark_index != -1) {
      associations(i) = best_landmark_index;
    } else if (observation_confidences[i] > this->_params_.new_landmark_confidence_gate) {
      associations(i) = -1;  // New landmark
    }
  }

  return associations;
}