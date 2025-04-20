#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "perception_sensor_lib/data_association/base_data_association.hpp"
#include "perception_sensor_lib/landmark_filter/parameters.hpp"

/**
 * @brief This class is meant to filter observations from perception to try to reduce the presence
 * of outliers
 *
 */
class LandmarkFilter {
protected:
  LandmarkFilterParameters _params_;
  std::shared_ptr<DataAssociationModel> _data_association_;

public:
  LandmarkFilter() = default;
  LandmarkFilter(LandmarkFilterParameters params,
                 std::shared_ptr<DataAssociationModel> data_association)
      : _params_(params), _data_association_(data_association) {}
  virtual ~LandmarkFilter() = default;
  /**
   * @brief This function filters observations
   *
   * @param new_observations observations that are considered new, relative to SLAM's map, in the
   * global reference frame
   * @return Eigen::VectorXd Each entry corresponds to an observation that should be added to the
   * state vector in the global reference frame
   */
  virtual Eigen::VectorXd filter(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
                                 const Eigen::VectorXd& observations,
                                 const Eigen::VectorXd& observation_confidences,
                                 const Eigen::VectorXi& associations) = 0;
};