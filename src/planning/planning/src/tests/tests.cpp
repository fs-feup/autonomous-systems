#include <gtest/gtest.h>

#include "../../include/planning/globalpathplanner.hpp"
#include "../../include/planning/localpathplanner.hpp"
#include "rclcpp/rclcpp.hpp"

using testing::Eq;

std::string filePrefix = rcpputils::fs::current_path().string();

TEST(GlobalPathPlanner, middlePath1) {
  Track* track = new Track();
  track->fillTrack("/planning/planning/files/map_test1.txt");

  GlobalPathPlanner* pathplanner = new GlobalPathPlanner(track);
  pathplanner->middlePath();

  vector<pair<float, float>> finalPath = pathplanner->getPath();

  vector<pair<float, float>> expected{
      make_pair(0, 0), make_pair(2, 0), make_pair(4, 0), make_pair(6, 0), make_pair(8, 0),
  };

  EXPECT_EQ(expected, finalPath);
}

TEST(GlobalPathPlanner, middlePath2) {
  Track* track = new Track();
  track->fillTrack("/planning/planning/files/map_test2.txt");

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
  Track* track = new Track();
  track->fillTrack("/planning/planning/files/map_test3.txt");

  GlobalPathPlanner* pathplanner = new GlobalPathPlanner(track);
  pathplanner->middlePath();

  vector<pair<float, float>> finalPath = pathplanner->getPath();

  vector<pair<float, float>> expected{
      make_pair(-0.5, 0),     make_pair(-0.5, 1),     make_pair(-0.875, 1.75),
      make_pair(-1.25, 2.25), make_pair(-2.25, 1.75),
  };

  EXPECT_EQ(expected, finalPath);
}

TEST(LocalPathPlanner, delauney1){
  Track* track = new Track();
  track->fillTrack("/planning/planning/files/map_test1.txt");

  LocalPathPlanner* pathplanner = new LocalPathPlanner();
  std::vector<Position> path;
  std::vector<Position*> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++)
    path.push_back(*pathPointers[i]);

  
  // std::vector<Position*> expected{Position(0, 0), Position(0, 1)}
  // preencher com as Positions que a função deveria retornar
  // EXPECT_EQ(expected, path);
  // naclass position é preciso definir um operator== para ele saber comparar. Há exemplos na net
}
