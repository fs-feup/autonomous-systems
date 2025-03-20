#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_set>

#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"

/**
 * @brief Data association implementation that uses the Malhanobis Distance only
 * as criterion to make observation matches.
 *
 */
class NearestNeighbour : public DataAssociationModel {
public:
  NearestNeighbour(const DataAssociationParameters& params) : DataAssociationModel(params) {}

  ~NearestNeighbour() = default;

  Eigen::VectorXi associate(const Eigen::VectorXd& state, const Eigen::SparseMatrix<double>& covariance,
                            const Eigen::VectorXd& observations,
                            const Eigen::VectorXd& observation_confidences) const override;
};