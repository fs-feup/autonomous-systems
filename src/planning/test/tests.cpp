#include <chrono>
#include <fstream>

#include "gtest/gtest.h"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/files.hpp"

using testing::Eq;

std::vector<Position> processTriangulations(std::string filename, std::string testname) {
  Track *track = new Track();
  track->fillTrack(filename);  // fill track with file data

  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<Position> path;

  double total_time = 0;
  int no_iters = 100;

  // One time iteration
  auto s0 = std::chrono::high_resolution_clock::now();

  std::vector<Position *> pathPointers = pathplanner->processNewArray(track);

  auto s1 = std::chrono::high_resolution_clock::now();

  double elapsed_time_zero_ms = std::chrono::duration<double, std::milli>(s1 - s0).count();
  std::cout << "\nDelaunay Triangulations processed in " << elapsed_time_zero_ms << " ms.\n";

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<Position *> pathPointers = pathplanner->processNewArray(track);

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    total_time += elapsed_time_ms;

    for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
  }

  std::ofstream measuresPath = openWriteFile("performance/exec_time/planning.csv");
  measuresPath << "planning,delaunay," << testname << "," << total_time / no_iters << ","
               << elapsed_time_zero_ms << "\n";
  measuresPath.close();

  std::cout << "\nAverage Delaunay Triangulations processed in " << total_time / no_iters
            << " ms.\n";

  return path;
}

void outlierCalculations(std::string filename, std::string testname) {
  Track *track = new Track();
  track->fillTrack(filename);  // fill track with file data

  // Remove outliers once
  auto s0 = std::chrono::high_resolution_clock::now();
  track->validateCones();
  auto s1 = std::chrono::high_resolution_clock::now();
  double elapsed_time_iter0_ms = std::chrono::duration<double, std::milli>(s1 - s0).count();
  std::cout << "Outliers removed in " << elapsed_time_iter0_ms << " ms\n";

  // Remove outliers no_iters times to get average
  int no_iters = 100;
  double total_time = 0;
  for (int i = 0; i < no_iters; i++) {
    track->reset();
    track->fillTrack(filename);

    auto t0 = std::chrono::high_resolution_clock::now();
    track->validateCones();
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_time_iter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_time += elapsed_time_iter_ms;
  }

  std::ofstream measuresPath = openWriteFile("performance/exec_time/planning.csv");
  measuresPath << "planning,outliers," << testname << "," << total_time / no_iters << ","
               << elapsed_time_iter0_ms << "\n";
  measuresPath.close();

  std::cout << "Outliers removed in average " << total_time / no_iters << " ms\n";
}

std::ostream &operator<<(std::ostream &os, const Position &p) {
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
  std::vector<Position> path = processTriangulations(filePath, "10points");

  std::vector<Position> expected{Position(1.5, 0),   Position(1.75, 1),    Position(2, 2),
                                 Position(2.5, 2.4), Position(3, 2.8),     Position(3.5, 3.15),
                                 Position(4, 3.5),   Position(4.25, 4.25), Position(4.5, 5)};

  // EXPECT_EQ(expected, path);
}

TEST(LocalPathPlanner, delauney100) {
  std::string filePath = "src/planning/tracks/map_100.txt";
  std::vector<Position> path = processTriangulations(filePath, "100points");
}

TEST(LocalPathPlanner, delauney250) {
  std::string filePath = "src/planning/tracks/map_250.txt";
  std::vector<Position> path = processTriangulations(filePath, "250points");
}

TEST(LocalPathPlanner, delauneyrng) {
  std::string filePath = "src/planning/tracks/map_250_rng.txt";
  std::vector<Position> path = processTriangulations(filePath, "250randompoints");
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
