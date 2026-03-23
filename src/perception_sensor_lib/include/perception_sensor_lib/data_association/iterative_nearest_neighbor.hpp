#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"

/**
 * @brief Data association implementation that first finds the transformation and then associates with the nearest neighbor
 *
 */
class IterativeNearestNeighbor : public DataAssociationModel {
public:
  IterativeNearestNeighbor(const DataAssociationParameters& params);

  ~IterativeNearestNeighbor() = default;

  Eigen::VectorXi associate(const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
                            const Eigen::MatrixXd& covariance,
                            const Eigen::VectorXd& observation_confidences,
                            const Eigen::Vector3d& pose) const override;

private:
  Eigen::VectorXd adjust_observations(const Eigen::VectorXd& observations,
                                      const Eigen::Vector3d& transform) const;
};
