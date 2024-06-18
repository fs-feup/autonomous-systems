// #include "kalman_filter/data_association.hpp"

// #include <gtest/gtest.h>

// #include <cmath>

// /**
//  * @brief Test case for when a landmarks is not in the state yet
//  */
// TEST(DATA_ASSOCIATION_MODEL, NEW_LANDMARK_1) {
//   // Arrange
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(5);
//   x_vector << 0, 0, 0, 0, 0;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   int landmark_index = data_association_model.match_cone(landmark_absolute, x_vector);

//   // Assert
//   EXPECT_EQ(landmark_index, -2);
// }

// /**
//  * @brief Test case for when a landmarks is not in the state yet
//  */
// TEST(DATA_ASSOCIATION_MODEL, NEW_LANDMARK_2) {
//   // Arrange
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(7);
//   x_vector << 0, 0, 0, 0, 0, 1, 2;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   int landmark_index = data_association_model.match_cone(landmark_absolute, x_vector);

//   // Assert
//   EXPECT_EQ(landmark_index, -2);
// }

// /**
//  * @brief Test case for when a landmarks is not in the state yet
//  */
// TEST(DATA_ASSOCIATION_MODEL, TOO_FAR_LANDMARK) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << 100, 100;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(7);
//   x_vector << 0, 0, 0, 0, 0, 1, 2;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   int landmark_index = data_association_model.match_cone(landmark_absolute, x_vector);

//   // Assert
//   EXPECT_EQ(landmark_index, -1);
// }

// /**
//  * @brief Test case for when a landmark has a perfect match
//  */
// TEST(DATA_ASSOCIATION_MODEL, PERFECT_MATCH) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << 10, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 10, 10, static_cast<float>(M_PI / 2), 0, 0, 1, 2, 10, 10;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   int landmark_index = data_association_model.match_cone(landmark_absolute, x_vector);

//   // Assert
//   EXPECT_EQ(landmark_index, 7);
// }

// /**
//  * @brief Test case for when a landmark has a near match
//  */
// TEST(DATA_ASSOCIATION_MODEL, NEAR_MATCH) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << 10, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 0, 0, static_cast<float>(M_PI / 2), 0, 0, 1, 2, 11, 10;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   int landmark_index = data_association_model.match_cone(landmark_absolute, x_vector);

//   // Assert
//   EXPECT_EQ(landmark_index, 7);
// }

// /**
//  * @brief Test case for when a landmark has a near mismatch
//  */
// TEST(DATA_ASSOCIATION_MODEL, NEAR_MISMATCH) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << 10, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 10, 10, static_cast<float>(M_PI / 2), 0, 0, 1, 2, 11, 10;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   int landmark_index = data_association_model.match_cone(landmark_absolute, x_vector);

//   // Assert
//   EXPECT_EQ(landmark_index, -2);
// }

// /**
//  * @brief Test case for when a landmark has a near match
//  */
// TEST(DATA_ASSOCIATION_MODEL, NEAR_MATCH_NEGATIVE) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << -20, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 4, 1, static_cast<float>(M_PI / 2), 0, 0, 1, 2, -20, 10;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   int landmark_index = data_association_model.match_cone(landmark_absolute, x_vector);

//   // Assert
//   EXPECT_EQ(landmark_index, 7);
// }

// /**
//  * @brief Test case for testing the valid match function - 0 delta
//  */
// TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_PERFECT_MATCH) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << -20, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 4, 1, static_cast<float>(M_PI / 2), 0, 0, 1, 2, -20, 10;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   bool result_a = data_association_model.valid_match(0, 0);
//   bool result_b = data_association_model.valid_match(0, 10);

//   // Assert
//   EXPECT_TRUE(result_a);
//   EXPECT_TRUE(result_b);
// }

// /**
//  * @brief Test case for testing the valid match function - some delta
//  */
// TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_NEAR_MATCH) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << -20, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 4, 1, static_cast<float>(M_PI / 2), 0, 0, 1, 2, -20, 10;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   bool result_a = data_association_model.valid_match(0.4, 0);
//   bool result_b = data_association_model.valid_match(1, 10);

//   // Assert
//   EXPECT_TRUE(result_a);
//   EXPECT_TRUE(result_b);
// }

// /**
//  * @brief Test case for testing the valid match function - big delta
//  */
// TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_FAILED_MATCH) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = 50.0;
//   float curvature = 8.0;
//   float initial_limit = 0.5;
//   landmark_absolute << -20, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 4, 1, static_cast<float>(M_PI / 2), 0, 0, 1, 2, -20, 10;
//   SimpleMaximumLikelihood data_association_model(limit_distance);
//   SimpleMaximumLikelihood::curvature_ = curvature;
//   SimpleMaximumLikelihood::initial_limit_ = initial_limit;

//   // Act
//   bool result_a = data_association_model.valid_match(0.6, 0);
//   bool result_b = data_association_model.valid_match(5, 10);

//   // Assert
//   EXPECT_FALSE(result_a);
//   EXPECT_FALSE(result_b);
// }

// /**
//  * @brief Test case for testing data association with negative parameters
//  */
// TEST(DATA_ASSOCIATION_MODEL, INVALID_ARGUMENTS) {
//   // Arrange
//   Eigen::Vector2f landmark_absolute = Eigen::Vector2f::Zero();
//   float limit_distance = -50.0;
//   float curvature = -8.0;
//   float initial_limit = -0.5;
//   landmark_absolute << -20, 10;
//   Eigen::VectorXf x_vector = Eigen::VectorXf::Zero(9);
//   x_vector << 4, 1, static_cast<float>(M_PI / 2), 0, 0, 1, 2, -20, 10;

//   // Act
//   try {
//     SimpleMaximumLikelihood data_association_model(limit_distance);
//     SimpleMaximumLikelihood::curvature_ = curvature;
//     SimpleMaximumLikelihood::initial_limit_ = initial_limit;
//     FAIL() << "Expected std::invalid_argument";
//   } catch (std::invalid_argument const& err) {
//     EXPECT_EQ(err.what(), std::string("Invalid parameters for SimpleMaximumLikelihood"));
//   } catch (...) {
//     FAIL() << "Expected std::invalid_argument";
//   }
// }