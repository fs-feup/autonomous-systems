#include "test_utils/utils.hpp"

/**
 * @brief simple scenario with few points in the path
 *
 */
TEST(VelocityPlanning, velocity0) {
  std::string file_path = "src/planning/tracks/velocity1.txt";
  VelocityPlanning velocity_planning;
  std::vector<common_lib::structures::PathPoint> final_path = ::path_from_file(file_path);
  velocity_planning.set_velocity(final_path);

  EXPECT_EQ((int)final_path.size(), 14);
  EXPECT_EQ(final_path.back().ideal_velocity, 3);
}

/**
 * @brief Check if the velocity is always greater than the minimum velocity
 *
 */
TEST(VelocityPlanning, velocity1) {
  std::string file_path = "src/planning/tracks/velocity1.txt";
  VelocityPlanning velocity_planning;
  std::vector<common_lib::structures::PathPoint> final_path = ::path_from_file(file_path);
  velocity_planning.set_velocity(final_path);

  for (const auto& point : final_path) {
    EXPECT_TRUE(point.ideal_velocity >= 3);
  }

}

/**
 * @brief test for a path with only one point
 *
 */
TEST(VelocityPlanning, velocity2) {
  std::string file_path = "src/planning/tracks/velocity2.txt";
  VelocityPlanning velocity_planning;
  std::vector<common_lib::structures::PathPoint> final_path = ::path_from_file(file_path);
  velocity_planning.set_velocity(final_path);

  EXPECT_EQ((int)final_path.size(), 1);
  EXPECT_EQ(final_path[0].ideal_velocity, 3);
}


/**
 * @brief test for a path with no points
 *
 */
TEST(VelocityPlanning, velocity3) {
  std::string file_path = "src/planning/tracks/velocity3.txt";
  VelocityPlanning velocity_planning;
  std::vector<common_lib::structures::PathPoint> final_path = ::path_from_file(file_path);
  velocity_planning.set_velocity(final_path);

  EXPECT_EQ((int)final_path.size(), 0);
}

/// Straight path, one metre per point.
static std::vector<common_lib::structures::PathPoint> straight_run(int points) {
  std::vector<common_lib::structures::PathPoint> path;
  path.reserve(points);
  for (int i = 0; i < points; i++) {
    common_lib::structures::PathPoint point;
    point.position.x = static_cast<double>(i);
    point.position.y = 0.0;
    path.push_back(point);
  }
  return path;
}

// minimum 3, desired 30, braking -5, lateral 10, longitudinal 5, planning enabled
static VelocityPlanningConfig acceleration_config() {
  return VelocityPlanningConfig(3.0, 30.0, -5.0, 10.0, 5.0, true);
}

/// Accelerates all the way to the braking point rather than levelling off early.
TEST(VelocityPlanning, AccelerationAcceleratesUntilTheBrakingDistance) {
  VelocityPlanning velocity_planning(acceleration_config());
  auto path = straight_run(150);
  velocity_planning.acceleration_velocity(path, 75.0);

  for (int i = 1; i <= 75; i++) {
    EXPECT_GE(path[i].ideal_velocity, path[i - 1].ideal_velocity) << "at index " << i;
  }
  // v^2 = v0^2 + 2*a*d = 9 + 2*5*75 = 759 -> 27.5 m/s, under the 30 cap.
  EXPECT_NEAR(path[75].ideal_velocity, std::sqrt(759.0), 1e-6);
}

/// Braking begins at the braking point rather than being deferred to the end of the path.
TEST(VelocityPlanning, AccelerationBrakesAsEarlyAsPossible) {
  VelocityPlanning velocity_planning(acceleration_config());
  auto path = straight_run(150);
  velocity_planning.acceleration_velocity(path, 75.0);

  EXPECT_LT(path[76].ideal_velocity, path[75].ideal_velocity);
  // At the full 5 m/s^2: v^2 = 759 - 2*5*1.
  EXPECT_NEAR(path[76].ideal_velocity, std::sqrt(749.0), 1e-6);

  for (int i = 76; i < static_cast<int>(path.size()); i++) {
    EXPECT_LE(path[i].ideal_velocity, path[i - 1].ideal_velocity) << "at index " << i;
  }
}

/// The profile commands a standstill rather than trailing off at the minimum velocity.
TEST(VelocityPlanning, AccelerationBrakesAllTheWayToZero) {
  VelocityPlanning velocity_planning(acceleration_config());
  // Stopping from sqrt(759) m/s at 5 m/s^2 needs 75.9 m, so the path has to be long enough to
  // contain it: 180 points is 179 m, leaving 104 m of braking room after the 75 m run.
  auto path = straight_run(180);
  velocity_planning.acceleration_velocity(path, 75.0);

  EXPECT_DOUBLE_EQ(path.back().ideal_velocity, 0.0);

  // Stays stopped: no point after the first zero recovers to the minimum velocity.
  bool stopped = false;
  for (const auto &point : path) {
    if (point.ideal_velocity == 0.0) {
      stopped = true;
    } else if (stopped) {
      ADD_FAILURE() << "velocity recovered to " << point.ideal_velocity << " after stopping";
      break;
    }
  }
  EXPECT_TRUE(stopped);
}

/// A path shorter than the run is all acceleration, with no braking phase yet.
TEST(VelocityPlanning, AccelerationShorterThanTheBrakingDistanceDoesNotBrake) {
  VelocityPlanning velocity_planning(acceleration_config());
  auto path = straight_run(30);
  velocity_planning.acceleration_velocity(path, 75.0);

  for (int i = 1; i < static_cast<int>(path.size()); i++) {
    EXPECT_GE(path[i].ideal_velocity, path[i - 1].ideal_velocity) << "at index " << i;
  }
}

/// stop() brakes to a standstill inside the lap but leaves the loop tail driveable, since those
/// indices are back at the start line where a car still finishing its lap sits.
TEST(VelocityPlanning, StopBrakesToZeroButLeavesTheLoopTailDriveable) {
  VelocityPlanning velocity_planning(acceleration_config());
  auto path = straight_run(150);
  velocity_planning.set_velocity(path);
  velocity_planning.stop(path, 20.0);

  const int stop_zone_end = 150 - 150 / 4;

  // Reaches a standstill: with the sign bug, stop() accelerated and never lowered a point.
  EXPECT_DOUBLE_EQ(path[stop_zone_end - 1].ideal_velocity, 0.0);

  // Tail still driveable, so a car finishing its lap is not commanded to stop dead.
  for (int i = stop_zone_end; i < 150; i++) {
    EXPECT_GT(path[i].ideal_velocity, 0.0) << "at index " << i;
  }
}
