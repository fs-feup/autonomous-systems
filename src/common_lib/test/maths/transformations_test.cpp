#include "common_lib/maths/transformations.hpp"

#include <gtest/gtest.h>

#include <random>

// Function to generate a random integer in the interval [a, b[
int generate_random_integer(const int a, const int b) {
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_int_distribution distr(a, b - 1);
  return distr(eng);
}

// Function to generate a random double in the interval [a, b[
double generate_random_double(const double a, const double b) {
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution distr(a, b);
  return distr(eng);
}

/**
 * @brief Trivial scenario of transformation from global to local coordinates
 * with no landmarks and trivial state
 *
 */
TEST(TransformGlobalToLocal, TestCase1) {
  const Eigen::Vector3d reference_frame(0, 0, 0);
  const Eigen::VectorXd global_landmarks = Eigen::VectorXd::Zero(0);
  const Eigen::VectorXd transformed_landmarks =
      common_lib::maths::global_to_local_coordinates(reference_frame, global_landmarks);
  EXPECT_EQ(transformed_landmarks.size(), 0);
}

/**
 * @brief Trivial scenario of transformation from global to local coordinates with no landmarks
 * and non trivial state
 *
 */
TEST(TransformGlobalToLocal, TestCase2) {
  const Eigen::Vector3d reference_frame(12, -2, 1.5);
  const Eigen::VectorXd global_landmarks = Eigen::VectorXd::Zero(0);
  const Eigen::VectorXd transformed_landmarks =
      common_lib::maths::global_to_local_coordinates(reference_frame, global_landmarks);
  EXPECT_EQ(transformed_landmarks.size(), 0);
}

/**
 * @brief test transformation from global to local coordinates with trivial state
 *
 */
TEST(TransformGlobalToLocal, TestCase3) {
  const Eigen::Vector3d reference_frame(0, 0, 0);
  Eigen::VectorXd global_landmarks = Eigen::VectorXd::Zero(10);
  global_landmarks << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Eigen::VectorXd expected_local_landmarks(10);
  expected_local_landmarks << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Eigen::VectorXd local_landmarks =
      common_lib::maths::global_to_local_coordinates(reference_frame, global_landmarks);
  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(local_landmarks(i), expected_local_landmarks(i), 1e-6);
  }
}

/**
 * @brief test transformation from global to local coordinates
 *
 */
TEST(TransformGlobalToLocal, TestCase4) {
  const Eigen::Vector3d reference_frame(5, -5, 1.1);
  Eigen::VectorXd global_landmarks = Eigen::VectorXd::Zero(10);
  global_landmarks << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Eigen::VectorXd expected_local_landmarks(10);
  expected_local_landmarks << 4.424067034727738, 6.740002290224783, 7.113673997701763,
      5.864779812953067, 9.803280960675789, 4.989557335681351, 12.492887923649814,
      4.114334858409635, 15.18249488662384, 3.239112381137919;
  Eigen::VectorXd local_landmarks =
      common_lib::maths::global_to_local_coordinates(reference_frame, global_landmarks);
  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(local_landmarks(i), expected_local_landmarks(i), 1e-6);
  }
}

/**
 * @brief Trivial scenario of transformation from local to global coordinates
 * with no landmarks and trivial state
 *
 */
TEST(TransformLocalToGlobal, TestCase1) {
  const Eigen::Vector3d reference_frame(0, 0, 0);
  const Eigen::VectorXd local_landmarks = Eigen::VectorXd::Zero(0);
  const Eigen::VectorXd transformed_landmarks =
      common_lib::maths::local_to_global_coordinates(reference_frame, local_landmarks);
  EXPECT_EQ(transformed_landmarks.size(), 0);
}

/**
 * @brief Test with no landmarks and non trivial state
 *
 */
TEST(TransformLocalToGlobal, TestCase2) {
  const Eigen::Vector3d reference_frame(5, -5, 1.1);
  Eigen::VectorXd local_landmarks = Eigen::VectorXd::Zero(0);
  Eigen::VectorXd global_landmarks =
      common_lib::maths::local_to_global_coordinates(reference_frame, local_landmarks);
  EXPECT_EQ(global_landmarks.size(), 0);
}

/**
 * @brief Test with trivial state and some landmarks
 *
 */
TEST(TransformLocalToGlobal, TestCase3) {
  const Eigen::Vector3d reference_frame(0, 0, 0);
  Eigen::VectorXd local_landmarks = Eigen::VectorXd::Zero(10);
  local_landmarks << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Eigen::VectorXd expected_global_landmarks(10);
  expected_global_landmarks << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Eigen::VectorXd global_landmarks =
      common_lib::maths::local_to_global_coordinates(reference_frame, local_landmarks);
  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(global_landmarks(i), expected_global_landmarks(i), 1e-6);
  }
}

/**
 * @brief Test transformation from local to global coordinates
 *
 */
TEST(TransformLocalToGlobal, TestCase4) {
  const Eigen::Vector3d reference_frame(5, -5, 1.1);
  Eigen::VectorXd local_landmarks = Eigen::VectorXd::Zero(10);
  local_landmarks << 4.424067034727738, 6.740002290224783, 7.113673997701763, 5.864779812953067,
      9.803280960675789, 4.989557335681351, 12.492887923649814, 4.114334858409635,
      15.18249488662384, 3.239112381137919;
  Eigen::VectorXd expected_global_landmarks(10);
  expected_global_landmarks << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Eigen::VectorXd global_landmarks =
      common_lib::maths::local_to_global_coordinates(reference_frame, local_landmarks);
  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(global_landmarks(i), expected_global_landmarks(i), 1e-6);
  }
}

/**
 * @brief Randomized scenario where coordinates are transformed from global to local and
 * then back to global
 *
 */
TEST(TransformGlobalANDLocal, TestCase1) {
  Eigen::Vector3d referencial;
  Eigen::VectorXd global_landmarks;
  Eigen::VectorXd transformed_landmarks;

  // Run 15 randomized tests
  for (int i = 0; i < 15; i++) {
    const int number_of_landmarks = ::generate_random_integer(1, 20);
    referencial << ::generate_random_double(-100, 100), ::generate_random_double(-100, 100),
        ::generate_random_double(-M_PI, M_PI);
    global_landmarks = Eigen::VectorXd::Zero(2 * number_of_landmarks);
    for (int j = 0; j < 2 * number_of_landmarks; j++) {
      global_landmarks(j) = ::generate_random_double(-100, 100);
    }
    transformed_landmarks =
        common_lib::maths::global_to_local_coordinates(referencial, global_landmarks);
    transformed_landmarks =
        common_lib::maths::local_to_global_coordinates(referencial, transformed_landmarks);
    for (int j = 0; j < 2 * number_of_landmarks; j++) {
      EXPECT_NEAR(global_landmarks(j), transformed_landmarks(j), 1e-6);
    }
  }
}

/**
 * @brief Randomized scenario where coordinates are transformed from local to global and
 * then back to local
 *
 */
TEST(TransformGlobalANDLocal, TestCase2) {
  Eigen::Vector3d referencial;
  Eigen::VectorXd local_landmarks;
  Eigen::VectorXd transformed_landmarks;

  // Run 15 randomized tests
  for (int i = 0; i < 15; i++) {
    const int number_of_landmarks = ::generate_random_integer(1, 20);
    referencial << ::generate_random_double(-100, 100), ::generate_random_double(-100, 100),
        ::generate_random_double(-M_PI, M_PI);
    local_landmarks = Eigen::VectorXd::Zero(2 * number_of_landmarks);
    for (int j = 0; j < 2 * number_of_landmarks; j++) {
      local_landmarks(j) = ::generate_random_double(-100, 100);
    }
    transformed_landmarks =
        common_lib::maths::local_to_global_coordinates(referencial, local_landmarks);
    transformed_landmarks =
        common_lib::maths::global_to_local_coordinates(referencial, transformed_landmarks);
    for (int j = 0; j < 2 * number_of_landmarks; j++) {
      EXPECT_NEAR(local_landmarks(j), transformed_landmarks(j), 1e-6);
    }
  }
}