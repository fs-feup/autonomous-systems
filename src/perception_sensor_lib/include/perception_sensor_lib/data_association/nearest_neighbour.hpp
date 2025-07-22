#pragma once
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

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
  /**
   * @brief Uses ICP to transform the observations to the landmarks frame. Used to make ICNN
   * reliable to bad pose estimates.
   *
   * @param landmarks landmarks in the form of [x1, y1, x2, y2, ...] in the global frame
   * @param observations observations in the form of [x1, y1, x2, y2, ...] in the global frame
   * @return Eigen::VectorXd Transformed observations in the form of [x1, y1, x2, y2, ...] in the
   * global frame
   */
  Eigen::VectorXd transform_points(const Eigen::VectorXd& landmarks,
                                   const Eigen::VectorXd& observations) const;

public:
  NearestNeighbour(const DataAssociationParameters& params) : DataAssociationModel(params) {}

  ~NearestNeighbour() = default;

  Eigen::VectorXi associate(const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
                            const Eigen::MatrixXd& covariance,
                            const Eigen::VectorXd& observation_confidences) const override;
};