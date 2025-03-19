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
class NearestNeighbor : public DataAssociationModel {
public:
  NearestNeighbor(const DataAssociationParameters& params);

  ~NearestNeighbor() = default;

  Eigen::VectorXi associate(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
                            const Eigen::VectorXd& observations,
                            const Eigen::VectorXd& observation_confidences) const override;
};