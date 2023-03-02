#include <iostream>

#include "include/pathplanner.hpp"

int main() {
  Track* track = new Track();
  track->fillTrack("files\\map_mock.txt");
  PathPlanner* pathplanner = new PathPlanner(track);
  pathplanner->middlePath();

  return 0;
}
