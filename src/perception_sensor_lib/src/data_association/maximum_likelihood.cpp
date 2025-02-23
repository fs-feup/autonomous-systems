#include "perception_sensor_lib/data_association/maximum_likelihood.hpp"

std::vector<int> MaximumLikelihood::associate(
    const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
    const Eigen::VectorXd& observations, const Eigen::VectorXd& observation_confidences) const {
  return std::vector<int>();
}