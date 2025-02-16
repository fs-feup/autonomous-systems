#include "planning/path_calculation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "utils/cone.hpp"

std::vector<Cone *> PathCalculation::process_delaunay_triangulations(
    std::pair<std::vector<Cone>, std::vector<Cone>> refined_cones,
    std::vector<PathPoint *> &mid_points) const {
  // merge left and right cones for next step
  // RCLCPP_WARN(rclcpp::get_logger("planning"), "Refined cones: %d blue, %d yellow",
  //            static_cast<int>(refined_cones.first.size()),
  //            static_cast<int>(refined_cones.second.size()));
  std::vector<Cone> cones;
  std::vector<Cone *> cones_with_neighbors;
  cones.reserve(refined_cones.first.size() + refined_cones.second.size());
  cones.insert(cones.end(), refined_cones.first.begin(), refined_cones.first.end());
  cones.insert(cones.end(), refined_cones.second.begin(), refined_cones.second.end());

  // Create a Delaunay triangulation
  DT dt;
  // Insert cones coordinates into the Delaunay triangulation
  for (const Cone &cone : cones) {
    cones_with_neighbors.push_back(new Cone(cone.position.x, cone.position.y));
    dt.insert(Point(cone.position.x, cone.position.y));
  }

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
    if (id_cone1 == -1 || id_cone2 == -1) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cone not found in triangulations");
      continue;
    }

    Cone *cone1 = cones_with_neighbors[id_cone1];
    Cone *cone2 = cones_with_neighbors[id_cone2];
    PathPoint *mid_point = new PathPoint((x1 + x2) / 2, (y1 + y2) / 2, cone1, cone2);
    mid_points.push_back(mid_point);

    cone1->neighbors.push_back(cone2);
    cone2->neighbors.push_back(cone1);
  }

  return cones_with_neighbors;
}

std::vector<PathPoint> PathCalculation::skidpad_path(std::vector<Cone> &cone_array,
                                                     common_lib::structures::Pose pose) {
  if (!path_orientation_corrected_) {
    std::string file = "./src/planning/src/utils/skidpad.txt";  // --------------------
    // open and read file line by line
    std::ifstream infile(file);
    std::string line;
    std::vector<PathPoint> hardcoded_path_;
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      double x, y, v;
      if (!(iss >> x >> y >> v)) {
        break;
      }  // error
      hardcoded_path_.push_back(PathPoint(x, y, v));
    }

    if (cone_array.size() < 4) {
      throw std::runtime_error("Insufficient cones to calculate path");  // --------------------
    }
    // sort the cones by distance
    std::sort(cone_array.begin(), cone_array.end(), [&pose](Cone &a, Cone &b) {
      return sqrt(pow((a.position.x - pose.position.x), 2) +
                  pow((a.position.y - pose.position.y), 2)) <
             sqrt(pow((b.position.x - pose.position.x), 2) +
                  pow((b.position.y - pose.position.y), 2));
    });

    // get the middle point between the two closest points
    PathPoint middle_closest =
        PathPoint((cone_array[0].position.x + cone_array[1].position.x) / 2,
                  (cone_array[0].position.y + cone_array[1].position.y) / 2, 0);

    // get the middle point between the two farthest points
    PathPoint middle_farthest =
        PathPoint((cone_array[2].position.x + cone_array[3].position.x) / 2,
                  (cone_array[2].position.y + cone_array[3].position.y) / 2, 0);

    // Find the slope of the line between the two middle points
    double dx = middle_farthest.position.x - middle_closest.position.x;  // --------------------
    double dy = middle_farthest.position.y - middle_closest.position.y;  // --------------------
    double slope = (dx != 0) ? (dy / dx) : 10000000000;                  // --------------------

    // calculate the angle
    double angle = atan(slope);

    // rotate all the points by the angle and add the origin point
    for (auto &point : hardcoded_path_) {
      double x = point.position.x;
      double y = point.position.y;
      point.position.x = x * cos(angle) - y * sin(angle) + middle_closest.position.x;
      point.position.y = x * sin(angle) + y * cos(angle) + middle_closest.position.y;
    }

    predefined_path_ = hardcoded_path_;
    path_orientation_corrected_ = true;
  }

  while (!predefined_path_.empty() &&
         pose.position.euclidean_distance(predefined_path_[0].position) < 1) {
    predefined_path_.erase(predefined_path_.begin());
  }

  // set it as the final path
  return std::vector<PathPoint>(predefined_path_.begin(), predefined_path_.begin() + 70);
}
