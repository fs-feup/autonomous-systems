#include "../include/planning/global_path_planner.hpp"

GlobalPathPlanner::GlobalPathPlanner(Track* track) : track(track) {}

void GlobalPathPlanner::middlePath() {
  int count = 0;
  int rightConesSize = track->getRightConesSize();
  for (int i = 0; i < track->getLeftConesSize(); i++) {
    Cone* leftCone = track->getLeftConeAt(i);
    float dist = track->getRightConeAt(count % rightConesSize)
                     ->getDistanceTo(leftCone);  // previous cone distance

    while (true) {
      Cone* candidateRightCone = track->getRightConeAt((count + 1) % rightConesSize);
      float new_dist = candidateRightCone->getDistanceTo(leftCone);  // next cone distance

      if (new_dist > dist)
        break;          // if new cone is not closer to the left one the previous,
                        // break(near cone found),
      dist = new_dist;  // else the new cone is now the candidate cone
      count++;
    }
    Cone* chosenRightCone = track->getRightConeAt(count % rightConesSize);
    // vector between left and right cones
    float crossTrackDistX = leftCone->getX() - chosenRightCone->getX();
    float crossTrackDistY = leftCone->getY() - chosenRightCone->getY();

    // calculate vector middle point
    float midPointX = leftCone->getX() - crossTrackDistX / 2;
    float midPointY = leftCone->getY() - crossTrackDistY / 2;

    finalPath.push_back(new Position(midPointX, midPointY));
  }

  for (size_t i = 0; i < finalPath.size(); i++) {
    cout << "x = " << finalPath[i]->getX() << " y = " << finalPath[i]->getY() << endl;
  }

  cout << "Middle path calculated with " << finalPath.size() << " points\n";
}

// void GlobalPathPlanner::writeFinalPath(const std::string &filePrefix) {
//   ofstream finalPathFile(filePrefix + "/planning/planning/files/finalPath.txt");

//   for (size_t i = 0; i < finalPath.size(); i++)
//     finalPathFile << finalPath[i]->getX() << " " << finalPath[i]->getY() << "\n";

//   finalPathFile.close();
// }

vector<pair<float, float>> GlobalPathPlanner::getPath() {
  vector<pair<float, float>> pathCopy;

  for (size_t i = 0; i < finalPath.size(); i++)
    pathCopy.push_back(make_pair(finalPath[i]->getX(), finalPath[i]->getY()));

  return pathCopy;
}
