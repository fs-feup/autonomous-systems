#pragma once
#include <Eigen/Dense>
#include <vector>

#include "perception_sensor_lib/data_association/parameters.hpp"
/**
 * @brief Data association models are responsible for associating landmarks with observations
 * given the current state, covariance, and confidences in the observations.
 *
 */
class DataAssociationModel {
protected:
  DataAssociationParameters _params_;

public:
  DataAssociationModel() = default;
  DataAssociationModel(DataAssociationParameters params) : _params_(params) {}
  virtual ~DataAssociationModel() = default;
  /**
   * @brief This function associates the landmarks with the observations
   *
   * @param state Vector of car's position and landmarks in the form of [car_x, car_y,
   * car_orientation, x1, y1, x2, y2, ...] all in the global frame
   * @param covariance Covariance matrix of the state vector
   * @param observations Observations in the form of [x1, y1, x2, y2, ...] in the car's frame
   * @param observation_confidences Confidence in the observations in the same order as the
   * observations
   * @return Eigen::VectorXi Each entry corresponds to an observation and contains the index of the
   * landmark that the observation is associated with in the state vector (x coordinate). If the
   * observation is considered new, the entry is -1. If the observation is considered an outlier,
   * the entry is -2.
   */
  virtual Eigen::VectorXi associate(const Eigen::VectorXd& state,
                                    [[maybe_unused]] const Eigen::MatrixXd& covariance,
                                    const Eigen::VectorXd& observations,
                                    const Eigen::VectorXd& observation_confidences) const = 0;
};