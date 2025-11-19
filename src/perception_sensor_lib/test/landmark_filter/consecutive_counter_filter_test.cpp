#include "perception_sensor_lib/landmark_filter/consecutive_counter_filter.hpp"

#include <gtest/gtest.h>

#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

/**
 * @brief Test with same observations 3 times and car still.
 *
 */
TEST(ConsecutiveCounterFilter, TestCase1) {
  Eigen::VectorXd observations(4);
  observations << 0.1, 0.2, 0.7, 0.8;
  Eigen::VectorXd confidences(2);
  confidences << 0.99, 0.99;
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params(50.0, 0.43, 0.8, 0.1, 0.1);
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  Eigen::VectorXd filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  filtered_observations = filter.filter(observations, confidences);
  ASSERT_EQ(filtered_observations.size(), 4);
  EXPECT_EQ(filtered_observations(0), 0.1);
  EXPECT_EQ(filtered_observations(1), 0.2);
  EXPECT_EQ(filtered_observations(2), 0.7);
  EXPECT_EQ(filtered_observations(3), 0.8);
}

/**
 * @brief Same test as above but with different order of observations.
 *
 */
TEST(ConsecutiveCounterFilter, TestCase2) {
  Eigen::VectorXd observations(4);
  Eigen::VectorXd confidences(2);
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params(50.0, 0.43, 0.8, 0.1, 0.1);
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  observations << 0.1, 0.2, 0.7, 0.8;
  confidences << 0.99, 0.99;
  Eigen::VectorXd filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.1, 0.2, 0.7, 0.8;
  confidences << 0.99, 0.99;
  filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.7, 0.8, 0.1, 0.2;
  confidences << 0.99, 0.99;
  filtered_observations = filter.filter(observations, confidences);
  ASSERT_EQ(filtered_observations.size(), 4);
  EXPECT_EQ(filtered_observations(0), 0.1);
  EXPECT_EQ(filtered_observations(1), 0.2);
  EXPECT_EQ(filtered_observations(2), 0.7);
  EXPECT_EQ(filtered_observations(3), 0.8);
}

/**
 * @brief Same test as above but one of the landmarks is not observed 3 times.
 *
 */
TEST(ConsecutiveCounterFilter, TestCase3) {
  Eigen::VectorXd observations(4);
  Eigen::VectorXd confidences(2);
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params(50.0, 0.43, 0.8, 0.1, 0.1);
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  observations << 0.1, 0.2, 0.7, 0.8;
  confidences << 0.99, 0.99;
  Eigen::VectorXd filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.1, 0.2, 0.7, 0.8;
  confidences << 0.99, 0.99;
  filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.7, 0.8, 2, 2.1;
  confidences << 0.99, 0.99;
  filtered_observations = filter.filter(observations, confidences);
  ASSERT_EQ(filtered_observations.size(), 2);
  EXPECT_EQ(filtered_observations(0), 0.7);
  EXPECT_EQ(filtered_observations(1), 0.8);
}

/**
 * @brief Same test as above but the car moves and observations change accordingly.
 *
 */
TEST(ConsecutiveCounterFilter, TestCase4) {
  Eigen::VectorXd observations(4);
  Eigen::VectorXd confidences(2);
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params(50.0, 0.43, 0.8, 0.1, 0.1);
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  observations << 0.1, 0.2, 0.7, 0.8;
  confidences << 0.99, 0.99;
  Eigen::VectorXd filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.1, 0.2, 0.7, 0.8;
  confidences << 0.99, 0.99;
  filtered_observations = filter.filter(observations, confidences);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.6980866604370328, 0.7972940119689251, 2.000155460474267, 2.0952219144232935;
  confidences << 0.99, 0.99;
  filtered_observations = filter.filter(observations, confidences);
  ASSERT_EQ(filtered_observations.size(), 2);
  EXPECT_EQ(filtered_observations(0), 0.7);
  EXPECT_EQ(filtered_observations(1), 0.8);
}