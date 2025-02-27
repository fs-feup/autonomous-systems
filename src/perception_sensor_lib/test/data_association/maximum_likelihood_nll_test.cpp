#include "perception_sensor_lib/data_association/maximum_likelihood_nll.hpp"

#include <gtest/gtest.h>

/**
 * @brief Test with mostly easy matches and a new landmark.
 *
 */
TEST(MaximumLikelihoodNLL, TestCase1) {
  // Arrange
  Eigen::VectorXd state(13);
  state << 0, 0, 0, 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << 4.5, 13.2, 34.2, -7.2, 3.1, 5.65, 6.86, 9.2, 1.7, 0.5;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(13, 13);
  covariance.setIdentity();
  covariance *= 0.1;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {-1, 3, 7, 9, 11};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Same case as TestCase1 but with rotation.
 *
 */
TEST(MaximumLikelihoodNLL, TestCase2) {
  // Arrange
  Eigen::VectorXd state(13);
  state << 0, 0, -0.7, 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << -5.0618836, 12.994896, 30.79597, 16.52538, -1.26881915231104, 6.31843318859,
      -0.67998532, 11.455881457, 0.97812287485, 1.47759116;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(13, 13);
  covariance.setIdentity();
  covariance *= 0.1;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {-1, 3, 7, 9, 11};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Same case as TestCase2 but with translation.
 *
 */
TEST(MaximumLikelihoodNLL, TestCase3) {
  // Arrange
  Eigen::VectorXd state(13);
  state << -2, 3, -0.7, 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << -1.59954619, 11.988805, 34.25830, 15.5192899677, 2.193518284, 5.3123420012,
      2.7823521, 10.449790270, 4.440460311, 0.47149997;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(13, 13);
  covariance.setIdentity();
  covariance *= 0.2;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {-1, 3, 7, 9, 11};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Only new landmarks
 *
 */
TEST(MaximumLikelihoodNLL, TestCase4) {
  // Arrange
  Eigen::VectorXd state(13);
  state << -2, 3, -0.7, 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(8);
  observations << -6.59954619, 7.988805, 14.25830, 5.5192899677, -2.193518284, 7.3123420012,
      12.7823521, 0.449790270;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(4);
  Eigen::MatrixXd covariance(13, 13);
  covariance.setIdentity();
  covariance *= 0.1;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {-1, -1, -1, -1};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Only new landmarks with low confidence
 *
 */
TEST(MaximumLikelihoodNLL, TestCase5) {
  // Arrange
  Eigen::VectorXd state(13);
  state << -2, 3, -0.7, 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(8);
  observations << -6.59954619, 7.988805, 14.25830, 5.5192899677, -2.193518284, 7.3123420012,
      12.7823521, 0.449790270;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd covariance(13, 13);
  covariance.setIdentity();
  covariance *= 0.1;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {-2, -2, -2, -2};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Only new landmarks with high confidence, high covariance and empty state
 *
 */
TEST(MaximumLikelihoodNLL, TestCase6) {
  // Arrange
  Eigen::VectorXd state(3);
  state << 4, -10, 2;
  Eigen::VectorXd observations(8);
  observations << -6.59954619, 7.988805, 14.25830, 5.5192899677, -2.193518284, 7.3123420012,
      12.7823521, 0.449790270;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(4);
  Eigen::MatrixXd covariance(3, 3);
  covariance.setIdentity();
  covariance *= 0.6;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {-1, -1, -1, -1};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Empty state and observations
 *
 */
TEST(MaximumLikelihoodNLL, TestCase7) {
  // Arrange
  Eigen::VectorXd state(3);
  state << 4, -10, 2;
  Eigen::VectorXd observations(0);
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(4);
  Eigen::MatrixXd covariance(3, 3);
  covariance.setIdentity();
  covariance *= 0.6;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  EXPECT_EQ(associations.size(), 0);
}

/**
 * @brief Same case as TestCase3 but covariance is different for some landmarks
 *
 */
TEST(MaximumLikelihoodNLL, TestCase8) {
  // Arrange
  Eigen::VectorXd state(13);
  state << -2, 3, -0.7, 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << -1.59954619, 11.988805, 34.25830, 15.5192899677, 2.193518284, 5.3123420012,
      2.7823521, 10.449790270, 4.440460311, 0.47149997;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(13, 13);
  covariance.setIdentity();
  covariance *= 0.2;
  covariance(5, 5) = 1000;  // Very high uncertainty, should match even though wasn't observed
  covariance(6, 6) = 1000;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {5, 3, 7, 9, 11};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}

/**
 * @brief Same case as TestCase3 but covariance is different for some landmarks
 *
 */
TEST(MaximumLikelihoodNLL, TestCase9) {
  // Arrange
  Eigen::VectorXd state(13);
  state << -2, 3, -0.7, 34.5, -7, 12.3, 4.5, 3.2, 5.6, 6.8, 9.1, 1.8, 0.4;
  Eigen::VectorXd observations(10);
  observations << -1.59954619, 11.988805, 34.25830, 15.5192899677, 2.193518284, 5.3123420012,
      2.7823521, 10.449790270, 4.440460311, 0.47149997;
  Eigen::VectorXd observation_confidences = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd covariance(13, 13);
  covariance.setIdentity();
  covariance *= 0.01;
  covariance(3, 3) = 0;  // Very low uncertainty, should not match even though was observed
  covariance(4, 4) = 0;
  DataAssociationParameters params;
  MaximumLikelihoodNLL ml(params);
  std::vector<int> expected_associations = {-1, -1, 7, 9, 11};
  // Act
  Eigen::VectorXi associations =
      ml.associate(state, covariance, observations, observation_confidences);
  // Assert
  for (int i = 0; i < static_cast<int>(associations.size()); ++i) {
    EXPECT_EQ(associations[i], expected_associations[i]);
  }
}
