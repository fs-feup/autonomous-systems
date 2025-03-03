#include "planning/path_calculation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "utils/cone.hpp"
using namespace std;

double PathCalculation::dfs_cost(int depth, MidPoint *previous, MidPoint *current, double maxcost) {
  if (depth == 0) {
    return 0;
  }
  // TODO: put in params
  double angle_weight = 10.0;
  double distance_weight = 5.0;
  double angle_exponent = 2.0;
  double distance_exponent = 0.998;

  double cost = 0;
  double res = maxcost * depth;
  for (MidPoint *next : current->close_points) {
    if (next == previous) {
      continue;
    }
    double distance = sqrt(pow(current->point.x() - next->point.x(), 2) +
                           pow(current->point.y() - next->point.y(), 2));
    double angle_with_previous =
        atan2(current->point.y() - previous->point.y(), current->point.x() - previous->point.x());
    double angle_with_next =
        atan2(next->point.y() - current->point.y(), next->point.x() - current->point.x());
    double angle = std::abs(angle_with_next - angle_with_previous);

    cost = pow(angle, angle_exponent) * angle_weight +
           pow(distance, distance_exponent) * distance_weight;

    if (cost > maxcost) {
      continue;
    }

    cost = cost + dfs_cost(depth - 1, current, next, maxcost);
    if (cost < res) {
      res = cost;
    }
  }

  return res;
}

std::vector<PathPoint> PathCalculation::no_coloring_planning(std::vector<Cone> &cone_array,
                                                             common_lib::structures::Pose pose) {
  std::unordered_map<Vertex, Vertex, VertexHash> vertexMap;
  std::list<MidPoint> midPoints;

  for (Cone cone : cone_array) {
    Vertex vertex = {Point(cone.position.x, cone.position.y), {}};
    vertexMap[vertex] = vertex;
  }

  DT dt;

  for (auto v : vertexMap) {
    dt.insert(v.first.point);
  }

  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    // Extract vertices' coordinates from both edges
    Vertex v1{it->first->vertex((it->second + 1) % 3)->point(), {}};
    Vertex v2{it->first->vertex((it->second + 2) % 3)->point(), {}};

    auto v1_it = vertexMap.find(v1);
    auto v2_it = vertexMap.find(v2);

    // Get pointers to the modifiable Vertex objects.
    Vertex *pv1 = &v1_it->second;
    Vertex *pv2 = &v2_it->second;

    // Compute the mid-point between the two vertices.
    MidPoint midPoint = {
        Point((v1.point.x() + v2.point.x()) / 2, (v1.point.y() + v2.point.y()) / 2),
        pv1,
        pv2,
        {}  // Initially empty close_points.
    };

    midPoints.push_back(midPoint);
    MidPoint &addedMidPoint = midPoints.back();

    // Link this midPoint with its adjacent vertices.
    addedMidPoint.v1->neighbors.push_back(&addedMidPoint);
    addedMidPoint.v2->neighbors.push_back(&addedMidPoint);
  }

  for (MidPoint &p : midPoints) {  // For each mid-point.
    Vertex *v1 = p.v1;
    Vertex *v2 = p.v2;

    for (MidPoint *m1 : v1->neighbors) {
      for (MidPoint *m2 : v2->neighbors) {
        if (m1 == m2) {
          continue;
        }

        if (m1->v1 == v1) {
          if (m2->v1 == v2 && m1->v2 == m2->v2) {
            p.close_points.push_back(m1);
            p.close_points.push_back(m2);
          } else if (m2->v2 == v2 && m1->v2 == m2->v1) {
            p.close_points.push_back(m1);
            p.close_points.push_back(m2);
          }
        } else if (m1->v2 == v1) {
          if (m2->v1 == v2 && m1->v1 == m2->v2) {
            p.close_points.push_back(m1);
            p.close_points.push_back(m2);
          } else if (m2->v2 == v2 && m1->v1 == m2->v1) {
            p.close_points.push_back(m1);
            p.close_points.push_back(m2);
          }
        }
      }
    }
  }

  MidPoint first;
  double min_dist = 10000;
  for (MidPoint &p : midPoints) {
    double dist =
        sqrt(pow(p.point.x() - pose.position.x, 2) + pow(p.point.y() - pose.position.y, 2));
    if (dist < min_dist) {
      min_dist = dist;
      first = p;
    }
  }

  double proj_distance = 1;  // 1 meter away from first point
  MidPoint projected = MidPoint{Point(first.point.x() + cos(pose.orientation) * proj_distance,
                                      first.point.y() + sin(pose.orientation) * proj_distance),
                                nullptr,
                                nullptr,
                                {}};

  min_dist = 10000;
  MidPoint second;
  for (MidPoint &p : midPoints) {
    double dist =
        sqrt(pow(p.point.x() - projected.point.x(), 2) + pow(p.point.y() - projected.point.y(), 2));
    if (dist < min_dist) {
      min_dist = dist;
      second = p;
    }
  }

  std::vector<MidPoint> path;
  path.push_back(first);
  path.push_back(second);

  bool planning = true;
  double max_cost = 500;
  int depth = 5;
  int n_points = 0;
  while (planning) {
    MidPoint *min_point = nullptr;
    double min_cost = 1000;
    for (MidPoint *p : path.back().close_points) {
      double cost = dfs_cost(depth, &path.back(), p, max_cost);
      if (cost < min_cost) {
        min_cost = cost;
        min_point = p;
      }
    }

    if (min_point == nullptr || n_points > 30) {
      planning = false;
    } else {
      path.push_back(*min_point);
      n_points++;
    }
  }

  std::vector<PathPoint> final_path;
  for (MidPoint &p : path) {
    final_path.push_back(PathPoint(p.point.x(), p.point.y()));
  }

  return final_path;
}

std::vector<PathPoint> PathCalculation::process_delaunay_triangulations(
    std::pair<std::vector<Cone>, std::vector<Cone>> refined_cones) const {
  // merge left and right cones for next step
  // RCLCPP_WARN(rclcpp::get_logger("planning"), "Refined cones: %d blue, %d yellow",
  //            static_cast<int>(refined_cones.first.size()),
  //            static_cast<int>(refined_cones.second.size()));
  std::vector<Cone> cones;
  cones.reserve(refined_cones.first.size() + refined_cones.second.size());
  cones.insert(cones.end(), refined_cones.first.begin(), refined_cones.first.end());
  cones.insert(cones.end(), refined_cones.second.begin(), refined_cones.second.end());

  std::vector<PathPoint> unordered_path;
  unordered_path.reserve(cones.size());

  // Create a Delaunay triangulation
  DT dt;
  // Insert cones coordinates into the Delaunay triangulation
  for (const Cone &cone : cones) {
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

    Cone cone1 = cones[id_cone1];
    Cone cone2 = cones[id_cone2];
    // If cones are from different sides
    if (cone1.color != cone2.color) {
      // Calculate the midpoint between the two cones
      double x_dist = cone2.position.x - cone1.position.x;
      double y_dist = cone2.position.y - cone1.position.y;
      double dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

      if (dist < config_.dist_threshold_) {
        PathPoint pt = PathPoint(cone1.position.x + x_dist / 2, cone1.position.y + y_dist / 2);
        unordered_path.push_back(pt);
      }
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Unordered path size: %d",
               static_cast<int>(unordered_path.size()));

  return unordered_path;
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
