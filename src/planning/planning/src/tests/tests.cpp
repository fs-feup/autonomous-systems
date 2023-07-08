#include "../../include/planning/global_path_planner.hpp"
#include "../../include/planning/local_path_planner.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

using testing::Eq;

/*TEST(GlobalPathPlanner, middlePath1) {
  std::string filePrefix = rcpputils::fs::current_path().string();
  Track* track = new Track();
  track->fillTrack(filePrefix + "/planning/planning/files/map_test1.txt");

  GlobalPathPlanner* pathplanner = new GlobalPathPlanner(track);
  pathplanner->middlePath();

  vector<pair<float, float>> finalPath = pathplanner->getPath();

  vector<pair<float, float>> expected{
      make_pair(0, 0), make_pair(2, 0), make_pair(4, 0), make_pair(6, 0), make_pair(8, 0),
  };

  EXPECT_EQ(expected, finalPath);
}

TEST(GlobalPathPlanner, middlePath2) {
  std::string filePrefix = rcpputils::fs::current_path().string();
  Track* track = new Track();
  track->fillTrack(filePrefix + "/planning/planning/files/map_test2.txt");

  GlobalPathPlanner* pathplanner = new GlobalPathPlanner(track);
  pathplanner->middlePath();

  vector<pair<float, float>> finalPath = pathplanner->getPath();

  vector<pair<float, float>> expected{
      make_pair(0.5, 0),   make_pair(0.5, 1),     make_pair(0.875, 1.75), make_pair(1.25, 2.25),
      make_pair(1.5, 2.5), make_pair(1.75, 2.25), make_pair(2.25, 1.75),
  };

  EXPECT_EQ(expected, finalPath);
}

TEST(GlobalPathPlanner, middlePath3) {
  std::string filePrefix = rcpputils::fs::current_path().string();
  Track* track = new Track();
  track->fillTrack(filePrefix + "/planning/planning/files/map_test3.txt");

  GlobalPathPlanner* pathplanner = new GlobalPathPlanner(track);
  pathplanner->middlePath();

  vector<pair<float, float>> finalPath = pathplanner->getPath();

  vector<pair<float, float>> expected{
      make_pair(-0.5, 0),     make_pair(-0.5, 1),     make_pair(-0.875, 1.75),
      make_pair(-1.25, 2.25), make_pair(-2.25, 1.75),
  };

  EXPECT_EQ(expected, finalPath);
}*/

/*std::ostream& operator<<(std::ostream& os, const Position& p) {
  return os << '(' << p.getX() << ',' << p.getY() << ')';
}*/

/*TEST(LocalPathPlanner, delauney1) {
  std::string filePrefix = rcpputils::fs::current_path().string();
  Track* track = new Track();
  track->fillTrack(filePrefix + "/planning/planning/tracks/map_test7.txt");

  LocalPathPlanner* pathplanner = new LocalPathPlanner();
  std::vector<Position> path;
  std::vector<Position*> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);

  // expected for map_test1:
  // std::vector<Position> expected{Position(0,0),Position(1,0), Position(2,0), Position(3,0),
  // Position(4,0), Position(5,0), Position(6,0), Position(7,0),Position(8,0)};

  //expected for map_test2:
  //std::vector<Position> expected{Position(0.5,0),Position(0.5,0.5), Position(0.5,1),
  //Position(0.75,1.5), Position(0.875,1.75), Position(1,2), Position(1.25,2.25),
  //Position(1.5,2.5),Position(1.75,2.25),Position(2,2),Position(2.25,1.75)};

  //expected for map_test6:
  std::vector<Position> expected{Position(1.5,0),Position(1.75,1), Position(2,2), Position(2.5,2.4),
  Position(3,2.8), Position(3.5,3.15), Position(4,3.5), Position(4.25,4.25),Position(4.5,5)};

  // dummy expected until sort done:
  //std::vector<Position> expected{Position(0, 0)};

  EXPECT_EQ(expected, path);

  // ============= Tmp write path to file ================
  std::string finalPath = filePrefix + "/planning/planning/tracks/finalPath.txt";
  std::ofstream finalPathFile(finalPath);
  for (size_t i = 0; i < pathPointers.size(); i++)
    finalPathFile << pathPointers[i]->getX() << " " << pathPointers[i]->getY() << "\n";
  finalPathFile.close();
}*/