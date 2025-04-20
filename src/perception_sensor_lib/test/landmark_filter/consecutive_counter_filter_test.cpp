#include "perception_sensor_lib/landmark_filter/consecutive_counter_filter.hpp"

#include <gtest/gtest.h>

#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"

/**
 * @brief Test with same observations 3 times and car still.
 *
 */
TEST(ConsecutiveCounterFilter, TestCase1) {
  Eigen::VectorXd observations(10);
  observations << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
  Eigen::VectorXi associations(5);
  associations << -1, 0, 1, -1, 3;
  Eigen::VectorXd confidences(5);
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params;
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  Eigen::VectorXd filtered_observations =
      filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0), observations,
                    confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  filtered_observations = filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0),
                                        observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  filtered_observations = filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0),
                                        observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 4);
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
  Eigen::VectorXd observations(10);
  Eigen::VectorXi associations(5);
  Eigen::VectorXd confidences(5);
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params;
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  observations << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
  associations << -1, 0, 1, -1, 3;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  Eigen::VectorXd filtered_observations =
      filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0), observations,
                    confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.3, 0.4, 0.1, 0.2, 0.7, 0.8, 0.5, 0.6, 0.9, 1.0;
  associations << 0, -1, -1, 1, 3;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  filtered_observations = filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0),
                                        observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.3, 0.4, 0.7, 0.8, 0.1, 0.2, 0.5, 0.6, 0.9, 1.0;
  associations << 0, -1, -1, 1, 3;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  filtered_observations = filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0),
                                        observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 4);
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
  Eigen::VectorXd observations(10);
  Eigen::VectorXi associations(5);
  Eigen::VectorXd confidences(5);
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params;
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  observations << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
  associations << -1, 0, 1, -1, 3;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  Eigen::VectorXd filtered_observations =
      filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0), observations,
                    confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.3, 0.4, 0.1, 0.2, 0.7, 0.8, 0.5, 0.6, 0.9, 1.0;
  associations << 0, -1, -1, 1, 3;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  filtered_observations = filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0),
                                        observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 0.3, 0.4, 0.7, 0.8, 0.5, 0.6, 0.9, 1.0, 2, 2.1;
  associations << 0, -1, 1, 3, -1;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  filtered_observations = filter.filter(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Zero(0, 0),
                                        observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 2);
  EXPECT_EQ(filtered_observations(0), 0.7);
  EXPECT_EQ(filtered_observations(1), 0.8);
}

/**
 * @brief Same test as above but the car moves and observations change accordingly.
 *
 */
TEST(ConsecutiveCounterFilter, TestCase4) {
  Eigen::VectorXd observations(10);
  Eigen::VectorXi associations(5);
  Eigen::VectorXd confidences(5);
  Eigen::VectorXd state(3);
  LandmarkFilterParameters params = LandmarkFilterParameters(3, 5);
  DataAssociationParameters data_association_params;
  std::shared_ptr<DataAssociationModel> data_association =
      std::make_shared<NearestNeighbor>(data_association_params);
  ConsecutiveCounterFilter filter(params, data_association);
  observations << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
  associations << -1, 0, 1, -1, 3;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  state << 0, 0, 0;
  Eigen::VectorXd filtered_observations =
      filter.filter(state, Eigen::MatrixXd::Zero(0, 0), observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << 1.3, -1.6, 1.1, -1.8, 1.7, -1.2, 1.5, -1.4, 1.9, -1.0;
  associations << 0, -1, -1, 1, 3;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  state << -1, 2, 0;
  filtered_observations =
      filter.filter(state, Eigen::MatrixXd::Zero(0, 0), observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 0);
  observations << -1.3, 1.6, -1.7, 1.2, -1.5, 1.4, -1.9, 1.0, -3.0, -0.1;
  associations << 0, -1, 1, 3, -1;
  confidences << 0.99, 0.99, 0.99, 0.99, 0.99;
  state << -1, 2, 3.14;
  filtered_observations =
      filter.filter(state, Eigen::MatrixXd::Zero(0, 0), observations, confidences, associations);
  EXPECT_EQ(filtered_observations.size(), 2);
  EXPECT_EQ(filtered_observations(0), 0.7);
  EXPECT_EQ(filtered_observations(1), 0.8);
}