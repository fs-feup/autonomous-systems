#include "planning/path_calculation.hpp"
#include "utils/cone.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

std::vector<PathPoint> PathCalculation::process_delaunay_triangulations(std::pair<std::vector<Cone>, std::vector<Cone>> refined_cones) {
  // merge left and right cones for next step
  std::vector<Cone> cones;
  cones.insert(cones.end(), refined_cones.first.begin(), refined_cones.first.end());
  cones.insert(cones.end(), refined_cones.second.begin(), refined_cones.second.end());

  std::vector<PathPoint> unordered_path;

  // Create a Delaunay triangulation
  DT dt;
  // Insert cones coordinates into the Delaunay triangulation
  for (size_t i = 0; i < cones.size(); i++)
    dt.insert(Point(cones[i].position.x, cones[i].position.y));

  // Process valid triangulations and add positions to unordered_path
  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    // Extract vertices' coordinates from both edges
    double x1 = it->first->vertex((it->second + 1) % 3)->point().x();
    double y1 = it->first->vertex((it->second + 1) % 3)->point().y();
    double x2 = it->first->vertex((it->second + 2) % 3)->point().x();
    double y2 = it->first->vertex((it->second + 2) % 3)->point().y();

    // Find corresponding cones for the vertices
    int id_cone1 = find_cone(cones, x1, y1);
    int id_cone2 = find_cone(cones, x2, y2);
    // Check both cones have been found
    if (id_cone1 == -1 || id_cone2 == -1){
     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cone not found in triangulations");
     continue;
    }

    Cone cone1 = cones[id_cone1];
    Cone cone2 = cones[id_cone2];
    // If cones are from different sides
    if ((cone1.color != cone2.color)) {
      // Calculate the midpoint between the two cones
      double xDist = cone2.position.x - cone1.position.x;
      double yDist = cone2.position.y - cone1.position.y;
      double dist = sqrt(pow(xDist, 2) + pow(yDist, 2));
      if (dist < config_.dist_threshold) {
        PathPoint pt = PathPoint(cone1.position.x + xDist / 2, cone1.position.y + yDist / 2);
        unordered_path.push_back(pt);
      }
    }
  }

  std::cout << unordered_path.size();

  return unordered_path;
}

