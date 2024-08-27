#include "kalman_filter/data_association.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "common_lib/structures/cone.hpp"
#include "kalman_filter/observation_models.hpp"

/**
 * @brief Test case for when a landmark is not in the state yet
 */
TEST(DATA_ASSOCIATION_MODEL, NEW_LANDMARK) {
  // Arrange
  float max_landmark_distance = 15.0f;
  MaxLikelihood data_association_model(max_landmark_distance);

  Eigen::VectorXf state_vector(8);
  state_vector << 0, 0, 0, 0, 0, 0, 1, 1;  // Initial state with one landmark at (5, 5)

  Eigen::MatrixXf covariance_matrix = Eigen::MatrixXf::Identity(8, 8);

  std::vector<common_lib::structures::Cone> perception_map = {
      {common_lib::structures::Cone{{10, 10}, common_lib::competition_logic::Color::BLUE, 1.0}}};

  std::vector<int> matched_ids;
  std::vector<Eigen::Vector2f> matched_cone_positions;
  std::vector<Eigen::Vector2f> new_features;

  Eigen::MatrixXf observation_noise_covariance_matrix = Eigen::MatrixXf::Identity(2, 2);
  ObservationModel observation_model(observation_noise_covariance_matrix);

  // Act
  data_association_model.associate_n_filter(perception_map, state_vector, covariance_matrix,
                                            matched_ids, matched_cone_positions, new_features,
                                            &observation_model);
  // Assert
  ASSERT_EQ(new_features.size(), 1);
  ASSERT_EQ(new_features[0](0), 10);
  ASSERT_EQ(new_features[0](1), 10);
}

/**
 * @brief Test case for when a landmark has a perfect match
 */
TEST(DATA_ASSOCIATION_MODEL, PERFECT_MATCH) {
  // Arrange
  float max_landmark_distance = 15.0f;
  MaxLikelihood data_association_model(max_landmark_distance);

  Eigen::VectorXf state_vector(8);
  state_vector << 0, 0, 0, 0, 0, 0, 3, 4;  // State vector with one landmark at (3, 4)

  Eigen::MatrixXf covariance_matrix = Eigen::MatrixXf::Identity(8, 8);

  std::vector<common_lib::structures::Cone> perception_map = {
      {common_lib::structures::Cone{{3.0, 4.0}, common_lib::competition_logic::Color::BLUE, 1.0}}};

  std::vector<int> matched_ids;
  std::vector<Eigen::Vector2f> matched_cone_positions;
  std::vector<Eigen::Vector2f> new_features;

  Eigen::MatrixXf observation_noise_covariance_matrix = Eigen::MatrixXf::Identity(2, 2);
  ObservationModel observation_model(observation_noise_covariance_matrix);

  // Act
  data_association_model.associate_n_filter(perception_map, state_vector, covariance_matrix,
                                            matched_ids, matched_cone_positions, new_features,
                                            &observation_model);

  // Assert
  ASSERT_EQ(matched_ids.size(), 1);
  ASSERT_EQ(matched_cone_positions.size(), 1);
  ASSERT_EQ(new_features.size(), 0);

  ASSERT_EQ(matched_ids[0],
            6);  // Assuming the first landmark in the state vector is the perfect match
  ASSERT_FLOAT_EQ(matched_cone_positions[0](0), 3.0);
  ASSERT_FLOAT_EQ(matched_cone_positions[0](1), 4.0);
}

/**
 * @brief Test case for when a landmark has a near match
 */
TEST(DATA_ASSOCIATION_MODEL, NEAR_MATCH) {
  // Arrange
  float max_landmark_distance = 15.0f;
  MaxLikelihood data_association_model(max_landmark_distance);

  Eigen::VectorXf state_vector(8);
  state_vector << 0, 0, 0, 0, 0, 0, 3.1, 4.1;  // State vector with one landmark at (3, 4)

  Eigen::MatrixXf covariance_matrix = Eigen::MatrixXf::Identity(8, 8);

  std::vector<common_lib::structures::Cone> perception_map = {
      {common_lib::structures::Cone{{3.0, 4.0}, common_lib::competition_logic::Color::BLUE, 1.0}}};

  std::vector<int> matched_ids;
  std::vector<Eigen::Vector2f> matched_cone_positions;
  std::vector<Eigen::Vector2f> new_features;

  Eigen::MatrixXf observation_noise_covariance_matrix = Eigen::MatrixXf::Identity(2, 2);
  ObservationModel observation_model(observation_noise_covariance_matrix);

  // Act
  data_association_model.associate_n_filter(perception_map, state_vector, covariance_matrix,
                                            matched_ids, matched_cone_positions, new_features,
                                            &observation_model);

  // Assert
  ASSERT_EQ(matched_ids.size(), 1);
  ASSERT_EQ(matched_cone_positions.size(), 1);
  ASSERT_EQ(new_features.size(), 0);

  ASSERT_EQ(matched_ids[0],
            6);  // Assuming the first landmark in the state vector is the perfect match
  ASSERT_NEAR(matched_cone_positions[0](0), 3.0, 0.1);
  ASSERT_NEAR(matched_cone_positions[0](1), 4.0, 0.1);
}

/**
 * @brief Test case for when a landmark has a near mismatch
 */
TEST(DATA_ASSOCIATION_MODEL, MISMATCH_NOT_NEW) {
  // Arrange
  float max_landmark_distance = 15.0f;
  MaxLikelihood data_association_model(max_landmark_distance);

  Eigen::VectorXf state_vector(8);
  state_vector << 0, 0, 0, 0, 0, 0, 7, 8;  // State vector with one landmark at (3, 4)

  Eigen::MatrixXf covariance_matrix = Eigen::MatrixXf::Identity(8, 8);

  std::vector<common_lib::structures::Cone> perception_map = {
      {common_lib::structures::Cone{{3.0, 4.0}, common_lib::competition_logic::Color::BLUE, 1.0}}};

  std::vector<int> matched_ids;
  std::vector<Eigen::Vector2f> matched_cone_positions;
  std::vector<Eigen::Vector2f> new_features;

  Eigen::MatrixXf observation_noise_covariance_matrix = Eigen::MatrixXf::Identity(2, 2);
  ObservationModel observation_model(observation_noise_covariance_matrix);

  // Act
  data_association_model.associate_n_filter(perception_map, state_vector, covariance_matrix,
                                            matched_ids, matched_cone_positions, new_features,
                                            &observation_model);

  // Assert

  ASSERT_EQ(matched_ids.size(), 0);
  ASSERT_EQ(matched_cone_positions.size(), 0);
  ASSERT_EQ(new_features.size(), 0);
}

// /**
//  * @brief Test case for testing the valid match function - 0 delta
//  */

// /**
//  * @brief Test case for testing the valid match function - some delta
//  */

// /**
//  * @brief Test case for testing the valid match function - big delta
//  */

// /**
//  * @brief Test case for testing data association with negative parameters
//  */