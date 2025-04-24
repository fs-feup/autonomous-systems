#include <set>

#include "perception_sensor_lib/landmark_filter/base_landmark_filter.hpp"

class ConsecutiveCounterFilter : public LandmarkFilter {
private:
  Eigen::VectorXd map;
  Eigen::VectorXi counter;

public:
  ConsecutiveCounterFilter(LandmarkFilterParameters params,
                           std::shared_ptr<DataAssociationModel> data_association)
      : LandmarkFilter(params, data_association), map(Eigen::VectorXd::Zero(0)) {}

  Eigen::VectorXd filter(const Eigen::VectorXd& new_observations,
                         const Eigen::VectorXd& new_observation_confidences) override;
  friend class ConsecutiveCounterFilter_TestCase1_Test;
};