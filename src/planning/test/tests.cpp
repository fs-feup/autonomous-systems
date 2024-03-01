#include <chrono>
#include <fstream>

#include "gtest/gtest.h"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/files.hpp"

using testing::Eq;

/**
 * @brief Test function for Delaunay Triangulations' efficiency.
 * Runs 100 times to get average execution time.
 *
 * @param filename path to the file that contains the data for testing
 * @param testname name of the test
 * @return path after Delaunay Triangulations
 */
std::vector<PathPoint> processTriangulations(std::string filename, std::string testname) {
  Track *track = new Track();
  track->fillTrack(filename);  // fill track with file data

  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<PathPoint> path;
  std::ofstream measuresPath = openWriteFile("src/performance/exec_time/planning.csv");
  double total_time = 0;
  int no_iters = 100;

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (i == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Delaunay Triangulations processed in %f ms.\n",
                   elapsed_time_ms);
      measuresPath << "planning, delaunay, " << testname << ", " << elapsed_time_ms;
    }

    total_time += elapsed_time_ms;

    for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
  }

  measuresPath << ", " << total_time / no_iters << "\n";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Delaunay Triangulations processed in %f ms.",
              (total_time / no_iters));

  return path;
}

/**
 * @brief Test function for outlier removal's efficiency.
 * Runs 100 times and calculates average duration.
 *
 * @param filename path to the file that contains the data for testing
 * @param testname name of the test
 */
void outlierCalculations(std::string filename, std::string testname) {
  Track *track = new Track();
  track->fillTrack(filename);  // fill track with file data

  // Remove outliers no_iters times to get average
  int no_iters = 100;
  double total_time = 0;
  std::ofstream measuresPath = openWriteFile("src/performance/exec_time/planning.csv");

  for (int i = 0; i < no_iters; i++) {
    track->reset();
    track->fillTrack(filename);
    auto t0 = std::chrono::high_resolution_clock::now();
    track->validateCones();
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_time_iter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (i == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Outliers removed in %f ms\n",
                   (elapsed_time_iter_ms));
      measuresPath << "planning, outliers, " << testname << ", " << elapsed_time_iter_ms;
    }
    total_time += elapsed_time_iter_ms;
  }

  measuresPath << ", " << total_time / no_iters << "\n";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Outliers removed in average %f ms.",
              (total_time / no_iters));
}

std::ostream &operator<<(std::ostream &os, const PathPoint &p) {
  return os << '(' << p.getX() << ',' << p.getY() << ')';
}

TEST(LocalPathPlanner, outliers) {
  std::string filePath = "src/planning/tracks/outlier_test1.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  track->validateCones();
  EXPECT_EQ(track->getLeftConesSize(),
            8);  // Both Have 9, 1 outlier should be removed
  EXPECT_EQ(track->getRightConesSize(), 8);
}

TEST(LocalPathPlanner, delauney10) {
  std::string filePath = "src/planning/tracks/map_10.txt";
  Track *track = new Track();
  track->fillTrack(filePath);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<PathPoint> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
  std::vector<PathPoint> expected{PathPoint(1.5, 0),   PathPoint(1.75, 1),    PathPoint(2, 2),
                                  PathPoint(2.5, 2.4), PathPoint(3, 2.8),     PathPoint(3.5, 3.15),
                                  PathPoint(4, 3.5),   PathPoint(4.25, 4.25), PathPoint(4.5, 5)};
  EXPECT_EQ(expected, path);
}

TEST(LocalPathPlanner, delauney100) {
  std::string filePath = "src/planning/tracks/map_100.txt";
  std::vector<PathPoint> path = processTriangulations(filePath, "100points");
}

TEST(LocalPathPlanner, delauney250) {
  std::string filePath = "src/planning/tracks/map_250.txt";
  std::vector<PathPoint> path = processTriangulations(filePath, "250points");
}

TEST(LocalPathPlanner, delauneyrng) {
  std::string filePath = "src/planning/tracks/map_250_rng.txt";
  std::vector<PathPoint> path = processTriangulations(filePath, "250randompoints");
}

TEST(LocalPathPlanner, delauneyoutliers0) {
  std::string filePath = "src/planning/tracks/map_250.txt";
  outlierCalculations(filePath, "250points_2outliers");
}

TEST(LocalPathPlanner, delauneyoutliers1) {
  std::string filePath = "src/planning/tracks/map_250_out10.txt";
  outlierCalculations(filePath, "250points_10outliers");
}

TEST(LocalPathPlanner, delauneyoutliers2) {
  std::string filePath = "src/planning/tracks/map_250_out25.txt";
  outlierCalculations(filePath, "250points_25outliers");
}

TEST(LocalPathPlanner, delauneyoutliers3) {
  std::string filePath = "src/planning/tracks/map_250_out50.txt";
  outlierCalculations(filePath, "250points_50outliers");
}
