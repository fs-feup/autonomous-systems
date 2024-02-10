#include "planning/global_path_planner.hpp"

#include <string>
#include <utility>
#include <vector>

GlobalPathPlanner::GlobalPathPlanner(Track *track) : track(track) {}

// void GlobalPathPlanner::writeFinalPath(const std::string &filePrefix) {
//   ofstream finalPathFile(filePrefix +
//   "/planning/planning/files/finalPath.txt");

//   for (size_t i = 0; i < finalPath.size(); i++)
//     finalPathFile << finalPath[i]->getX() << " " << finalPath[i]->getY() <<
//     "\n";

//   finalPathFile.close();
// }

std::vector<std::pair<float, float>> GlobalPathPlanner::getPath() {
  std::vector<std::pair<float, float>> pathCopy;

  for (size_t i = 0; i < finalPath.size(); i++)
    pathCopy.push_back(std::make_pair(finalPath[i]->getX(), finalPath[i]->getY()));

  return pathCopy;
}
