#include "planning/local_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

LocalPathPlanner::LocalPathPlanner() : track() {}

void LocalPathPlanner::setOrientation(float theta) { orientation = theta; }

bool LocalPathPlanner::vector_direction(Position* p1, Position* p2, float prev_vx, float prev_vy) {
  float vx = p2->getX() - p1->getX();
  float vy = p2->getY() - p1->getY();

  return (vx * prev_vx + vy * prev_vy) > 0;
}

std::vector<Position*> LocalPathPlanner::processNewArray(Track* cone_array) {
  std::vector<std::pair<Position*, bool>> unordered_path;

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
      unordered_path.push_back(std::make_pair(position, false));
    }
  }

  std::vector<Position*> final_path;
  Position* p1 = new Position(0, 0);
  size_t iter_number = 1;
  float vx = 1;
  float vy = 0;

  while (iter_number <= unordered_path.size()) {
    float min_dist = MAXFLOAT;
    size_t min_index = 0;

    for (size_t i = 0; i < unordered_path.size(); i++) {
      Position* p2 = unordered_path[i].first;
      if (unordered_path[i].second == false && vector_direction(p1, p2, vx, vy)) {
        float new_dist = p1->getDistanceTo(p2);
        if (new_dist < min_dist) {
          min_dist = new_dist;
          min_index = i;
        }
      }
    }
    iter_number++;
    unordered_path[min_index].second = true;

    vx = unordered_path[min_index].first->getX() - p1->getX();
    vy = unordered_path[min_index].first->getY() - p1->getY();

    p1 = unordered_path[min_index].first;
    final_path.push_back(unordered_path[min_index].first);
  }

  this->track.reset();
  return final_path;
}