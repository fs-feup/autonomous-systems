#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"

/**
 * @brief Data association implementation that uses the Malhanobis Distance only
 * as criterion to make observation matches.
 *
 */
class MaximumLikelihoodMD : public DataAssociationModel {
  Eigen::Matrix2d observation_noise_covariance_matrix_;

public:
  MaximumLikelihoodMD(const DataAssociationParameters& params) : DataAssociationModel(params) {
    observation_noise_covariance_matrix_ = Eigen::Matrix2d::Zero();
    observation_noise_covariance_matrix_(0, 0) = params.observation_x_noise;
    observation_noise_covariance_matrix_(1, 1) = params.observation_y_noise;
  }

  ~MaximumLikelihoodMD() = default;

  Eigen::VectorXi associate(const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
                            const Eigen::MatrixXd& covariance,
                            const Eigen::VectorXd& observation_confidences) const override;
};