#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

#include <gtest/gtest.h>

/**
 * @brief Test with mostly easy matches and a new landmark.
 *
 */
TEST(NearestNeighbor, TestCase1) {
  // Arrange
  Eigen::VectorXd landmarks(10);
  landmarks << 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << 4.5, 13.2, 34.2, -7.2, 3.1, 5.65, 6.86, 9.2, 1.7, 0.5;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(10, 10);
  DataAssociationParameters params(50.0, 0.43, 0.8, 0.1, 0.1);
  NearestNeighbor ml(params);
  std::vector<int> expected_associations = {-1, 0, 4, 6, 8};
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Same case as TestCase1 but with rotation.
 *
 */
TEST(NearestNeighbor, TestCase2) {
  // Arrange
  Eigen::VectorXd landmarks(10);
  landmarks << 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << 4.499999722610841, 13.199999626032849, 34.19999913867148, -7.200000784733975,
      3.0999999999972934, 5.649999999996787, 6.859999998027547, 9.200000001049675,
      1.699999998811323, 0.49999999845647614;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(10, 10);
  DataAssociationParameters params(50.0, 0.43, 0.8, 0.1, 0.1);
  NearestNeighbor ml(params);
  std::vector<int> expected_associations = {-1, 0, 4, 6, 8};
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Same case as TestCase2 but with translation.
 *
 */
TEST(NearestNeighbor, TestCase3) {
  // Arrange
  Eigen::VectorXd landmarks(10);
  landmarks << 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << 4.499999823221495, 13.199999786278871, 34.19999419521099, -7.1999951106971025,
      3.1000000000117893, 5.649999999968996, 6.859999985817916, 9.200000011828031,
      1.6999999970212407, 0.49999999653619254;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(10, 10);
  DataAssociationParameters params(50.0, 0.43, 0.8, 0.1, 0.1);
  std::vector<int> expected_associations = {-1, 0, 4, 6, 8};
  NearestNeighbor ml(params);
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Only new landmarks with high confidence
 *
 */
TEST(NearestNeighbor, TestCase4) {
  // Arrange
  Eigen::VectorXd landmarks(10);
  landmarks << 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(8);
  observations << -1.9010818621517096, 13.361719473329373, 12.460973577144308, -1.9640632387881682,
      1.033044730121015, 10.00589092620211, 8.066244986285891, -4.8905987333937615;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(4);
  Eigen::MatrixXd covariance(10, 10);
  DataAssociationParameters params(50.0, 0.43, 0.8, 0.1, 0.1);
  NearestNeighbor ml(params);
  std::vector<int> expected_associations = {-1, -1, -1, -1};
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Only new landmarks with low confidence
 *
 */
TEST(NearestNeighbor, TestCase5) {
  // Arrange
  Eigen::VectorXd landmarks(10);
  landmarks << 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(8);
  observations << -1.9010818621517096, 13.361719473329373, 12.460973577144308, -1.9640632387881682,
      1.033044730121015, 10.00589092620211, 8.066244986285891, -4.8905987333937615;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd covariance(10, 10);
  DataAssociationParameters params(50.0, 0.43, 0.8, 0.1, 0.1);
  NearestNeighbor ml(params);
  std::vector<int> expected_associations = {-2, -2, -2, -2};
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Only new landmarks with high confidence, high covariance and empty landmarks
 *
 */
TEST(NearestNeighbor, TestCase6) {
  // Arrange
  Eigen::VectorXd landmarks(0);
  Eigen::VectorXd observations(8);
  observations << -6.59954619, 7.988805, 14.25830, 5.5192899677, -2.193518284, 7.3123420012,
      12.7823521, 0.449790270;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(4);
  Eigen::MatrixXd covariance(0, 0);
  DataAssociationParameters params(50.0, 0.43, 0.8, 0.1, 0.1);
  NearestNeighbor ml(params);
  std::vector<int> expected_associations = {-1, -1, -1, -1};
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Empty landmarks and observations
 *
 */
TEST(NearestNeighbor, TestCase7) {
  // Arrange
  Eigen::VectorXd landmarks(0);
  Eigen::VectorXd observations(0);
  Eigen::VectorXd observation_confidences(0);
  Eigen::MatrixXd covariance(0, 0);
  DataAssociationParameters params(50.0, 1.5, 0.8, 0.1, 0.1);
  NearestNeighbor ml(params);
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  EXPECT_EQ(associations.size(), 0);
}

TEST(NearestNeighbor, TestCase8) {
  // Arrange
  Eigen::VectorXd landmarks(28);
  landmarks << 1.395321, 1.702726, 4.554022, -1.237558, 8.210190, 4.143576, 3.203739, 4.371034,
      6.414619, 6.287142, 10.785306, 2.572242, 7.224374, -3.087130, 13.088638, -7.904575, 13.266725,
      2.447038, 4.708611, 1.970104, 9.944443, -1.026403, 5.617912, -8.881818, 12.886412, -0.762993,
      7.856627, -13.420066;
  Eigen::VectorXd observations(30);
  observations << 1.415478, 1.678782, 4.581321, -1.253966, 3.242400, 4.364834, 8.236903, 4.121514,
      4.720761, 1.951072, 6.461193, 6.287533, 10.826837, 2.560291, 9.981289, -1.036438, 5.655921,
      -8.897340, 20.887318, -0.297938, 13.148567, -7.931385, 9.667393, -14.61736, 18.244286,
      4.775482, 7.306300, -3.099958, 16.092053, -1.154598;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(15);
  Eigen::MatrixXd covariance(10, 10);
  DataAssociationParameters params(50.0, 0.43, 0.8, 0.1, 0.1);
  NearestNeighbor ml(params);
  std::vector<int> expected_associations = {0, 2, 6, 4, 18, 8, 10, 20, 22, -1, 14, -1, -1, 12, -1};
  // Act
  Eigen::VectorXi associations =
      ml.associate(landmarks, observations, covariance, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}