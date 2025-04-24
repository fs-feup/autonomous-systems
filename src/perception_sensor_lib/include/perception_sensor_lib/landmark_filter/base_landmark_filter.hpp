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
   * @brief This function receives a new set of observations and returns a filtered set of landmarks
   * in global coordinates that are ready to be added to be added to the map
   *
   * @param observations Observations in the form of [x1, y1, x2, y2, ...] in the global frame
   * @param observation_confidences Confidence in the observations in the same order as the
   * observations
   * @return Eigen::VectorXd the filtered observations in the form of [x1, y1, x2, y2, ...] in the
   * global frame
   */
  virtual Eigen::VectorXd filter(const Eigen::VectorXd& new_observations,
                                 const Eigen::VectorXd& new_observation_confidences) = 0;
};