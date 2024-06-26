#include "test_utils/utils.hpp"

/**
 * @brief simple scenario only with left cones but with 4 significant outliers
 *
 */
TEST(Outliers, outliers_simple) {
  std::string file_path = "src/planning/tracks/outlier_test1.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  auto n1_left = static_cast<int>(track.first.size());
  auto n1_right = static_cast<int>(track.second.size());
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 31.1480);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 0);
  auto n2_left = static_cast<int>(track.first.size());
  auto n2_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, 36);
  EXPECT_EQ(n2_left, 36);
  EXPECT_EQ(n1_right, 0);
  EXPECT_EQ(n2_right, 0);
}

/**
 * @brief simple case in which there are cones only in the left size
 * and no significant outliers
 *
 */
TEST(Outliers, outlier_left_only) {
  std::string file_path = "src/planning/tracks/outlier_test2.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  auto n1_left = static_cast<int>(track.first.size());
  auto n1_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, 12);
  EXPECT_EQ(n1_right, 0);
  track = outliers.approximate_cones_with_spline(track);
  n1_left = static_cast<int>(track.first.size());
  n1_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, 12);
  EXPECT_EQ(n1_right, 0);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 2.772);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 0);
  auto n2_left = static_cast<int>(track.first.size());
  auto n2_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, 0);
  EXPECT_EQ(n2_right, 0);
}

/**
 * @brief scenario with large track but few outliers
 *
 */
TEST(Outliers, map250_out10) {
  std::string file_path = "src/planning/tracks/map_250_out10.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  auto n1_left = static_cast<int>(track.first.size());
  auto n1_right = static_cast<int>(track.second.size());
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 5.5149999);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 4.901);
  auto n2_left = static_cast<int>(track.first.size());
  auto n2_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}

/**
 * @brief scenario with large track and moderate number of outliers
 *
 */
TEST(Outliers, map250_out25) {
  std::string file_path = "src/planning/tracks/map_250_out25.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  auto n1_left = static_cast<int>(track.first.size());
  auto n1_right = static_cast<int>(track.second.size());
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 5.5100002);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 6.152);
  auto n2_left = static_cast<int>(track.first.size());
  auto n2_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}

/**
 * @brief scenario with large track and abundant outliers
 *
 */
TEST(Outliers, map250_out50) {
  std::string file_path = "src/planning/tracks/map_250_out50.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  auto n1_left = static_cast<int>(track.first.size());
  auto n1_right = static_cast<int>(track.second.size());
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 5.4310002);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 7.928);
  auto n2_left = static_cast<int>(track.first.size());
  auto n2_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}
/**
 * @brief scenario with large track that barely has outliers (only 2)
 *
 */
TEST(Outliers, map250) {
  std::string file_path = "src/planning/tracks/map_250.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  auto n1_left = static_cast<int>(track.first.size());
  auto n1_right = static_cast<int>(track.second.size());
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 4.6900001);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 4.5019999);
  auto n2_left = static_cast<int>(track.first.size());
  auto n2_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}

/**
 * @brief simplest scenario with cones forming a line and one outlier
 *
 */
TEST(Outliers, distance_next_outliers) {
  std::string file_path = "src/planning/tracks/distance_to_next.txt";
  auto track = track_from_file(file_path);
  auto n1_left = static_cast<int>(track.first.size());
  auto n1_right = static_cast<int>(track.second.size());
  Outliers outliers;
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 0);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 1.483);
  auto n2_left = static_cast<int>(track.first.size());
  auto n2_right = static_cast<int>(track.second.size());
  EXPECT_EQ(n1_left, 0);
  EXPECT_EQ(n2_left, 0);
  EXPECT_EQ(n1_right, 15);
  EXPECT_EQ(n2_right, 15);
}