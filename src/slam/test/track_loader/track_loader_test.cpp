#include "track_loader/track_loader.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>

TEST(AddLandmarksTest, EmptyList) {
  Eigen::VectorXd track(0);
  YAML::Node list = YAML::Load("[]");
  int coneCounter = 0;
  add_landmarks(track, list);
  EXPECT_EQ(track.size(), 0);
}

TEST(AddLandmarksTest, SingleLandmark) {
  Eigen::VectorXd track(0);
  YAML::Node list = YAML::Load("[{position: [1.0, 2.0]}]");
  int coneCounter = 0;
  add_landmarks(track, list);
  ASSERT_EQ(track.size(), 2);
  EXPECT_DOUBLE_EQ(track[0], 1.0);
  EXPECT_DOUBLE_EQ(track[1], 2.0);
}

TEST(AddLandmarksTest, MultipleLandmarks) {
  Eigen::VectorXd track(0);
  YAML::Node list =
      YAML::Load("[{position: [1.0, 2.0]}, {position: [3.0, 4.0]}, {position: [5.0, 6.0]}]");
  int coneCounter = 0;
  add_landmarks(track, list);
  ASSERT_EQ(track.size(), 6);
  EXPECT_DOUBLE_EQ(track[0], 1.0);
  EXPECT_DOUBLE_EQ(track[1], 2.0);
  EXPECT_DOUBLE_EQ(track[2], 3.0);
  EXPECT_DOUBLE_EQ(track[3], 4.0);
  EXPECT_DOUBLE_EQ(track[4], 5.0);
  EXPECT_DOUBLE_EQ(track[5], 6.0);
}

TEST(AddLandmarksTest, AppendsToExistingTrack) {
  Eigen::VectorXd track(2);
  track << 9.0, 8.0;
  YAML::Node list = YAML::Load("[{position: [1.0, 2.0]}, {position: [3.0, 4.0]}]");
  int coneCounter = 0;
  add_landmarks(track, list);
  ASSERT_EQ(track.size(), 6);
  EXPECT_DOUBLE_EQ(track[0], 9.0);
  EXPECT_DOUBLE_EQ(track[1], 8.0);
  EXPECT_DOUBLE_EQ(track[2], 1.0);
  EXPECT_DOUBLE_EQ(track[3], 2.0);
  EXPECT_DOUBLE_EQ(track[4], 3.0);
  EXPECT_DOUBLE_EQ(track[5], 4.0);
}

TEST(AddLandmarksTest, HandlesNegativeCoordinates) {
  Eigen::VectorXd track(0);
  YAML::Node list = YAML::Load("[{position: [-1.5, -2.5]}]");
  int coneCounter = 0;
  add_landmarks(track, list);
  ASSERT_EQ(track.size(), 2);
  EXPECT_DOUBLE_EQ(track[0], -1.5);
  EXPECT_DOUBLE_EQ(track[1], -2.5);
}

TEST(LoadMapTest, LoadsAccelerationYamlWithoutFailure) {
  Eigen::Vector3d start_position;
  Eigen::VectorXd track;
  load_map("src/slam/test/track_loader/skidpad.yaml", start_position, track);
  std::string track_string = "";
  for (int i = 0; i < track.size() / 2; ++i) {
    track_string +=
        "(" + std::to_string(track[2 * i]) + ", " + std::to_string(track[2 * i + 1]) + "), ";
  }
  RCLCPP_INFO(rclcpp::get_logger("slam"), "Track: %s", track_string.c_str());
}
