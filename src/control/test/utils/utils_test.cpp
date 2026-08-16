#include "control/include/utils/utils.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

using namespace common_lib::structures;

// Helper function to create PathPointArray from CSV file
std::vector<custom_interfaces::msg::PathPoint> create_path_msg(std::string track_file) {
  std::string track_file_path = "../../src/control/test/assets/" + track_file + ".csv";
  std::ifstream trackFile(track_file_path);

  std::vector<custom_interfaces::msg::PathPoint> pathpoint_array;

  std::string line;

  std::getline(trackFile, line);  // Skip the first line

  while (std::getline(trackFile, line)) {
    std::istringstream iss(line);
    std::string x, y, v;
    if (std::getline(iss, x, ',') && std::getline(iss, y, ',') && std::getline(iss, v)) {
      // Create a PathPoint and add it to the PathPointArray
      custom_interfaces::msg::PathPoint point;
      point.x = std::stod(x);  // string to double
      point.y = std::stod(y);
      point.v = std::stod(v);
      pathpoint_array.push_back(point);
    }
  }
  trackFile.close();
  return pathpoint_array;
};

/**
 * @brief Test Point Solver - get_closest_point()
 */
TEST(PointSolverTests, Test_update_closest_point_1) {
  auto pathpoint_array = create_path_msg("track1");
  ControlParameters params;
  params.pure_pursuit_lookahead_gain_ = 0.0;
  params.pure_pursuit_lookahead_minimum_ = 0.0;
  params.first_last_max_dist_ = 0.0;
  Position expected_point = Position(46.5, -12.37);
  int expected_id = 76;

  auto [path, rear_axis_point, closest_point_velocity] =
      get_closest_point(pathpoint_array, Position(47.0, -13.0));

  EXPECT_EQ(path.x, expected_point.x);
  EXPECT_EQ(path.y, expected_point.y);
  EXPECT_EQ(rear_axis_point, expected_id);
}

/**
 * @brief Test Point Solver - update_lookahead_point()
 */

TEST(PointSolverTests, Test_update_lookahead_point_1) {
  auto pathpoint_array = create_path_msg("track1");

  Position rear_axis_ = Position(47.0, -13.0);
  int closest_point_id = 76;
  Position expected_point = Position(48.576, -10.447);

  auto [result_point, result_velocity, result_error] =
      get_lookahead_point(pathpoint_array, closest_point_id, 3.0, rear_axis_, 1.0);

  EXPECT_NEAR(result_point.x, expected_point.x, 0.01);
  EXPECT_NEAR(result_point.y, expected_point.y, 0.01);
  EXPECT_FALSE(result_error);
}

// Straight path along x, one point per metre.
static std::vector<custom_interfaces::msg::PathPoint> straight_path(
    const std::vector<double>& velocities) {
  std::vector<custom_interfaces::msg::PathPoint> path;
  for (size_t i = 0; i < velocities.size(); i++) {
    custom_interfaces::msg::PathPoint point;
    point.x = static_cast<double>(i);
    point.y = 0.0;
    point.v = velocities[i];
    path.push_back(point);
  }
  return path;
}

TEST(InterpolatedVelocityTests, InterpolatesBetweenTwoPoints) {
  const auto path = straight_path({10.0, 20.0, 30.0});
  EXPECT_NEAR(get_interpolated_velocity(path, 1, Position(1.25, 0.0)), 22.5, 1e-9);
}

TEST(InterpolatedVelocityTests, InterpolatesOnTheSegmentBehindTheClosestPoint) {
  const auto path = straight_path({10.0, 20.0, 30.0});
  EXPECT_NEAR(get_interpolated_velocity(path, 1, Position(0.75, 0.0)), 17.5, 1e-9);
}

TEST(InterpolatedVelocityTests, LateralOffsetDoesNotChangeTheSetpoint) {
  const auto path = straight_path({10.0, 20.0, 30.0});
  EXPECT_NEAR(get_interpolated_velocity(path, 1, Position(1.5, 3.0)), 25.0, 1e-9);
}

TEST(InterpolatedVelocityTests, CarBeforeThePathTakesTheFirstVelocity) {
  const auto path = straight_path({10.0, 20.0, 30.0});
  // No point behind the car: hold the first point's velocity rather than extrapolate backwards.
  EXPECT_NEAR(get_interpolated_velocity(path, 0, Position(-5.0, 0.0)), 10.0, 1e-9);
  EXPECT_NEAR(get_interpolated_velocity(path, 1, Position(-5.0, 0.0)), 10.0, 1e-9);
}

TEST(InterpolatedVelocityTests, CarPastThePathTakesTheLastVelocity) {
  const auto path = straight_path({10.0, 20.0, 30.0});
  EXPECT_NEAR(get_interpolated_velocity(path, 2, Position(9.0, 0.0)), 30.0, 1e-9);
}

TEST(InterpolatedVelocityTests, ExactlyOnAPointReturnsThatPointsVelocity) {
  const auto path = straight_path({10.0, 20.0, 30.0});
  EXPECT_NEAR(get_interpolated_velocity(path, 1, Position(1.0, 0.0)), 20.0, 1e-9);
}

TEST(InterpolatedVelocityTests, DegenerateInputs) {
  const auto path = straight_path({10.0, 20.0, 30.0});
  EXPECT_NEAR(get_interpolated_velocity(path, -1, Position(0.0, 0.0)), 0.0, 1e-9);
  EXPECT_NEAR(get_interpolated_velocity({}, 0, Position(0.0, 0.0)), 0.0, 1e-9);
  EXPECT_NEAR(get_interpolated_velocity(straight_path({7.0}), 0, Position(3.0, 0.0)), 7.0, 1e-9);
  EXPECT_NEAR(get_interpolated_velocity(path, 99, Position(2.0, 0.0)), 30.0, 1e-9);
}

TEST(InterpolatedVelocityTests, RepeatedPointsDoNotDivideByZero) {
  std::vector<custom_interfaces::msg::PathPoint> path;
  for (int i = 0; i < 3; i++) {
    custom_interfaces::msg::PathPoint point;
    point.x = point.y = 0.0;
    point.v = 5.0 + i;
    path.push_back(point);
  }
  EXPECT_NEAR(get_interpolated_velocity(path, 1, Position(1.0, 1.0)), 6.0, 1e-9);
}
