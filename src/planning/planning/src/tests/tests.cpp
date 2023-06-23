#include "../../include/planning/global_path_planner.hpp"
#include "../../include/planning/local_path_planner.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

using testing::Eq;

TEST(LocalPathPlanner, delauney1) {
  std::string filePrefix = rcpputils::fs::current_path().string();
  Track* track = new Track();
  track->fillTrack(filePrefix + "/planning/planning/files/map_test1.txt");

  LocalPathPlanner* pathplanner = new LocalPathPlanner();
  std::vector<Position> path;
  std::vector<Position*> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);

  // std::vector<Position*> expected{Position(0, 0), Position(0, 1)}
  // preencher com as Positions que a função deveria retornar
  // EXPECT_EQ(expected, path);
  // naclass position é preciso definir um operator== para ele saber comparar. Há exemplos na net
}
