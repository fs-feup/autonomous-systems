#include <set>

#include "perception_sensor_lib/landmark_filter/base_landmark_filter.hpp"

class ConsecutiveCounterFilter : public LandmarkFilter {
private:
  Eigen::VectorXd map;
  Eigen::VectorXi counter;

public:
  ConsecutiveCounterFilter(LandmarkFilterParameters params,
                           std::shared_ptr<DataAssociationModel> data_association)
      : LandmarkFilter(params, data_association), map(Eigen::VectorXd::Zero(3)) {}

  Eigen::VectorXd filter(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance,
                         const Eigen::VectorXd& observations,
                         const Eigen::VectorXd& observation_confidences,
                         const Eigen::VectorXi& associations);
  friend class ConsecutiveCounterFilter_TestCase1_Test;
};