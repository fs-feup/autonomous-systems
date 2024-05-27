#include "planning/local_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

LocalPathPlanner::LocalPathPlanner() : track() {}

bool LocalPathPlanner::vector_direction(PathPoint *p1, PathPoint *p2, float prev_vx,
                                        float prev_vy) {
  float vx = p2->getX() - p1->getX();
  float vy = p2->getY() - p1->getY();

  return (vx * prev_vx + vy * prev_vy) > 0;
}

std::vector<PathPoint *> LocalPathPlanner::processNewArray(Track *cone_array) {
  std::vector<std::pair<PathPoint *, bool>> unordered_path;
  std::vector<PathPoint *> return_result;

  // Loop through left cones and add them to the track
  for (int i = 0; i < cone_array->getLeftConesSize(); i++) {
    this->track.setCone(cone_array->getLeftConeAt(i));
  }
  // Loop through right cones and add them to the track
  for (int i = 0; i < cone_array->getRightConesSize(); i++) {
    this->track.setCone(cone_array->getRightConeAt(i));
  }
  // Create a Delaunay triangulation
  DT dt;
  // Insert left cones' coordinates into the Delaunay triangulation
  for (int i = 0; i < track.getLeftConesSize(); i++) {
    Cone *lCone = track.getLeftConeAt(i);
    dt.insert(Point(lCone->getX(), lCone->getY()));
  }
  // Insert right cones' coordinates into the Delaunay triangulation
  for (int i = 0; i < track.getRightConesSize(); i++) {
    Cone *rCone = track.getRightConeAt(i);
    dt.insert(Point(rCone->getX(), rCone->getY()));
  }

  // Process valid triangulations and add positions to unordered_path
  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    // Extract vertices' coordinates from both edges
    float x1 = it->first->vertex((it->second + 1) % 3)->point().x();
    float y1 = it->first->vertex((it->second + 1) % 3)->point().y();
    float x2 = it->first->vertex((it->second + 2) % 3)->point().x();
    float y2 = it->first->vertex((it->second + 2) % 3)->point().y();

    // Find corresponding cones for the vertices
    Cone *cone1 = track.findCone(x1, y1);
    Cone *cone2 = track.findCone(x2, y2);

    // Check conditions for valid triangulation
    // both cones are not null and one of them is left and the other is right
    if (cone1 != nullptr && cone2 != nullptr && (cone1->getId() % 2 != cone2->getId() % 2)) {
      // Calculate the midpoint between the two cones
      float xDist = cone2->getX() - cone1->getX();
      float yDist = cone2->getY() - cone1->getY();
      float dist = sqrt(pow(xDist, 2) + pow(yDist, 2));
      if (dist < DELAUNAY_DIST_THRESHOLD) {
        PathPoint *position = new PathPoint(cone1->getX() + xDist / 2, cone1->getY() + yDist / 2);
        return_result.push_back(position);
        unordered_path.push_back(std::make_pair(position, false));
      }
    }
  }

  this->track.reset();
  return return_result;
}