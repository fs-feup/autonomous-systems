#include "perception_sensor_lib/data_association/iterative_nearest_neighbor.hpp"

#include <gtest/gtest.h>

#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

namespace {

int count_associations(const Eigen::VectorXi& associations) {
  int num_associations = 0;
  for (int idx = 0; idx < associations.size(); ++idx) {
    if (associations(idx) >= 0) {
      ++num_associations;
    }
  }

  return num_associations;
}

}  // namespace

TEST(IterativeNearestNeighbor, RecoversAssociationsAfterRigidOffset) {
  Eigen::VectorXd landmarks(8);
  landmarks << 1.0, 1.0, 4.0, 1.0, 1.0, 4.0, 4.0, 4.0;

  Eigen::VectorXd observations(12);
  observations << 1.411159575345, 0.558455930679, 4.365582834382, 1.079400463680,
      0.890215042344, 3.512879189716, 3.844638301381, 4.033823722717, 7.089276602762,
      8.667647445433, -2.237856394359, 3.976742409727;

  Eigen::VectorXd observation_confidences(6);
  observation_confidences << 1.0, 1.0, 1.0, 1.0, 1.0, 0.2;

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(8, 8);
  DataAssociationParameters params(10.0, 0.12, 0.8, 0.1, 0.1);

  NearestNeighbor nearest_neighbor(params);
  const Eigen::VectorXi raw_associations =
      nearest_neighbor.associate(landmarks, observations, covariance, observation_confidences,
                                 Eigen::Vector3d::Zero());
  EXPECT_EQ(count_associations(raw_associations), 0);

  IterativeNearestNeighbor iterative_nearest_neighbor(params);
  const Eigen::VectorXi associations =
      iterative_nearest_neighbor.associate(landmarks, observations, covariance,
                                           observation_confidences, Eigen::Vector3d::Zero());

  std::vector<int> expected_associations = {0, 2, 4, 6, -1, -2};
  for (int idx = 0; idx < associations.size(); ++idx) {
    EXPECT_EQ(associations(idx), expected_associations[idx]);
  }
}

TEST(IterativeNearestNeighbor, FallsBackToNearestNeighborWithSingleSeedMatch) {
  Eigen::VectorXd landmarks(2);
  landmarks << 2.0, -1.0;

  Eigen::VectorXd observations(4);
  observations << 2.04, -0.98, 7.0, 7.0;

  Eigen::VectorXd observation_confidences(2);
  observation_confidences << 1.0, 1.0;

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(2, 2);
  DataAssociationParameters params(10.0, 0.1, 0.8, 0.1, 0.1);

  IterativeNearestNeighbor iterative_nearest_neighbor(params);
  const Eigen::VectorXi associations =
      iterative_nearest_neighbor.associate(landmarks, observations, covariance,
                                           observation_confidences, Eigen::Vector3d::Zero());

  std::vector<int> expected_associations = {0, -1};
  for (int idx = 0; idx < associations.size(); ++idx) {
    EXPECT_EQ(associations(idx), expected_associations[idx]);
  }
}
