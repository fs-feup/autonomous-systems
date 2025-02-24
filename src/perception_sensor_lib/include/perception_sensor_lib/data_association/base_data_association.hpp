#pragma once
#include <Eigen/Dense>
#include <vector>

#include "perception_sensor_lib/data_association/parameters.hpp"

class DataAssociationModel {
protected:
  DataAssociationParameters params_;

public:
  DataAssociationModel(DataAssociationParameters params) : params_(params) {}
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
   * @return std::vector<int> Each entry corresponds to an observation and contains the index of the
   * landmark that the observation is associated with in the state vector (x coordinate). If the
   * observation is considered new, the entry is -1. If the observation is considered an outlier,
   * the entry is -2.
   */
  virtual std::vector<int> associate(const Eigen::VectorXd& state,
                                     const Eigen::MatrixXd& covariance,
                                     const Eigen::VectorXd& observations,
                                     const Eigen::VectorXd& observation_confidences) const = 0;
};