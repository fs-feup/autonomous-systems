#pragma once

#include "perception_sensor_lib/data_association/base_data_association.hpp"

class MaximumLikelihood : public DataAssociationModel {
public:
  MaximumLikelihood() = default;
  ~MaximumLikelihood() = default;

  std::vector<int> associate(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
                             const Eigen::VectorXd& observations,
                             const Eigen::VectorXd& observation_confidences) const override;
};