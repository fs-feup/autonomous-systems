#include "../include/planning/local_path_planner.hpp"

#include <cmath>
#include <map>
#include <utility>
#include <vector>

LocalPathPlanner::LocalPathPlanner() : track() {}

void LocalPathPlanner::setOrientation(float theta) { orientation = theta; }

bool LocalPathPlanner::vectorDirection(Position* p1, Position* p2) {
  float vx = p2->getX() - p1->getX();
  float vy = p2->getY() - p1->getY();

  float angle = std::atan2(vy, vx) * 180 / M_PI;  // convert to degrees

  float diff = std::abs(orientation - angle);

  return (diff < 90 || diff > 270);
}

std::vector<Position*> LocalPathPlanner::processNewArray(Track* cone_array) {
  std::vector<std::pair<Position*, bool>> unorderedPath;
  for (int i = 0; i < cone_array->getLeftConesSize(); i++)
    this->track.setCone(cone_array->getLeftConeAt(i));

  for (int i = 0; i < cone_array->getRightConesSize(); i++)
    this->track.setCone(cone_array->getRightConeAt(i));

  DT dt;

  for (int i = 0; i < track.getLeftConesSize(); i++) {
    Cone* lCone = track.getLeftConeAt(i);
    dt.insert(Point(lCone->getX(), lCone->getY()));
  }

  for (int i = 0; i < track.getRightConesSize(); i++) {
    Cone* rCone = track.getRightConeAt(i);
    dt.insert(Point(rCone->getX(), rCone->getY()));
  }

  size_t startIndex = 0;

  // Select the valid triangulations and add them to the map

  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    float x1 = it->first->vertex((it->second + 1) % 3)->point().x();
    float y1 = it->first->vertex((it->second + 1) % 3)->point().y();
    float x2 = it->first->vertex((it->second + 2) % 3)->point().x();
    float y2 = it->first->vertex((it->second + 2) % 3)->point().y();

    Cone* cone1 = track.findCone(x1, y1);
    Cone* cone2 = track.findCone(x2, y2);

    if (cone1 != nullptr && cone2 != nullptr && cone1->getId() % 2 != cone2->getId() % 2) {
      float xDist = cone2->getX() - cone1->getX();
      float yDist = cone2->getY() - cone1->getY();
      Position* position = new Position(cone1->getX() + xDist / 2, cone1->getY() + yDist / 2);

      // store start position
      if ((cone1->getId() == 0 && cone2->getId() == 1) ||
          (cone1->getId() == 1 && cone2->getId() == 0)) {
        startIndex = unorderedPath.size();
        unorderedPath.push_back(std::make_pair(position, true));
      } else {
        unorderedPath.push_back(std::make_pair(position, false));
      }
    }
  }

  Position* startPos = unorderedPath[startIndex].first;
  std::vector<Position*> finalPath = {startPos};

  float minDist = MAXFLOAT;
  size_t minIndex = 0;
  for (size_t j = 0; j < unorderedPath.size(); j++) {
    Position* p2 = unorderedPath[j].first;
    if (unorderedPath[j].second == false && j != startIndex) {
      // checks if vector is pointing to the same side of the car orientation
      if (vectorDirection(startPos, p2)) {
        float newDist = startPos->getDistanceTo(p2);
        if (newDist < minDist) {
          minDist = newDist;
          minIndex = j;
        }
      }
    }
  }

  finalPath.push_back(unorderedPath[minIndex].first);
  unorderedPath[minIndex].second = true;
    
  size_t i = minIndex;
  size_t iterCount = 2;
  while (iterCount < unorderedPath.size()) {
    Position* p1 = unorderedPath[i].first;
    minDist = MAXFLOAT;
    minIndex = 0;
    //std::cout << p1->getX() << " " << p1->getY() << " " << i << "\n";
    for (size_t j = 0; j < unorderedPath.size(); j++) {
      Position* p2 = unorderedPath[j].first;
      if (unorderedPath[j].second == false && j != i) {
        float newDist = p1->getDistanceTo(p2);
        if (newDist < minDist) {
          minDist = newDist;
          minIndex = j;
        }
      }
    }
    i = minIndex;
    iterCount++;
    unorderedPath[minIndex].second = true;
    finalPath.push_back(unorderedPath[minIndex].first);
  }
  // delete(cone_array);

  this->track.reset();

  return finalPath;
}

// float LocalPathPlanner::euclideanDist(Position* p1, Position* p2) {
//   return sqrt(pow(p2->getX() - p1->getX(), 2) + pow(p2->getY() - p1->getY(), 2));
// }