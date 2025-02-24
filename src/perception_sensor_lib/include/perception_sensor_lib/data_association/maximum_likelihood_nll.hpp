#pragma once
#include <memory>

#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"

/**
 * @brief Data association implementation that uses the Negative Log-Likehood
 * as criterion to make observation matches.
 *
 */
class MaximumLikelihoodNLL : public DataAssociationModel {
  Eigen::Matrix2d observation_noise_covariance_matrix_;

public:
  MaximumLikelihoodNLL(const DataAssociationParameters& params) : DataAssociationModel(params) {
    observation_noise_covariance_matrix_ = Eigen::Matrix2d::Zero();
    observation_noise_covariance_matrix_(0, 0) = params_.observation_x_noise;
    observation_noise_covariance_matrix_(1, 1) = params_.observation_y_noise;
  }

  ~MaximumLikelihoodNLL() = default;

  std::vector<int> associate(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
                             const Eigen::VectorXd& observations,
                             const Eigen::VectorXd& observation_confidences) const override;
};