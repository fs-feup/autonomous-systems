#include <set>

#include "perception_sensor_lib/landmark_filter/base_landmark_filter.hpp"
#define EQUALITY_TOLERANCE 1e-3

class ConsecutiveCounterFilter : public LandmarkFilter {
private:
  Eigen::VectorXd map;
  Eigen::VectorXi counter;

public:
  ConsecutiveCounterFilter(LandmarkFilterParameters params,
                           std::shared_ptr<DataAssociationModel> data_association)
      : LandmarkFilter(params, data_association), map(Eigen::VectorXd::Zero(0)) {}

  /**
   * @brief This function receives a new set of observations and returns a filtered set of landmarks
   * in global coordinates that are ready to be added to be added to the map
   * @details This filter keeps track of how many consecutive times an observation has been seen.
   * If an observation has been seen more than minimum_observation_count_ times consecutively, it is
   * added to the map and returned as a new landmark. If an observation is not seen in a given call
   * to filter, its counter is reset.
   * @param observations Observations in the form of [x1, y1, x2, y2, ...] in the global frame
   * @param observation_confidences Confidence in the observations in the same order as the
   * observations
   * @return Eigen::VectorXd the filtered observations in the form of [x1, y1, x2, y2, ...] in the
   * global frame
   */
  Eigen::VectorXd filter(const Eigen::VectorXd& observations,
                         const Eigen::VectorXd& observation_confidences,
                         Eigen::VectorXi& associations) override;

  void delete_landmarks(const Eigen::VectorXd& some_landmarks) override;
  friend class ConsecutiveCounterFilter_TestCase1_Test;
};