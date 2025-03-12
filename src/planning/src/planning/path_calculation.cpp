#include "planning/path_calculation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "utils/cone.hpp"
using namespace std;

std::pair<double, PathCalculation::MidPoint*> PathCalculation::dfs_cost(int depth, MidPoint* previous,
                                                       MidPoint* current, double maxcost) {
  if (depth == 0) {
    return {0, current};  // Return current point if depth is 0
  }

  double min_cost = this->config_.max_cost_ * this->config_.search_depth_;
  MidPoint* min_point = current;  // Default to current point

  for (MidPoint* next : current->close_points) {
    // Avoid revisiting the previous point
    if (next == previous) {
      continue;
    }

    // Distance calculation
    double distance = sqrt(pow(current->point.x() - next->point.x(), 2) +
                           pow(current->point.y() - next->point.y(), 2));

    // Angle calculation
    double angle_with_previous =
        atan2(current->point.y() - previous->point.y(), current->point.x() - previous->point.x());
    double angle_with_next =
        atan2(next->point.y() - current->point.y(), next->point.x() - current->point.x());

    // Normalize angle to be between 0 and π
    double angle = std::abs(angle_with_next - angle_with_previous);
    if (angle > M_PI) angle = 2 * M_PI - angle;

    // Local cost calculation
    double local_cost = pow(angle, this->config_.angle_exponent_) * this->config_.angle_gain_+
                        pow(distance, this->config_.distance_exponent_) * this->config_.distance_gain_;

    // Skip if local cost exceeds maximum allowed cost
    if (local_cost > maxcost) {
      continue;
    }

    // Recursive cost calculation
    std::pair<double, MidPoint*> recursive_result = dfs_cost(depth - 1, current, next, maxcost);

    // Total cost calculation
    double total_cost = local_cost + recursive_result.first;

    // Update minimum cost and corresponding point
    if (total_cost < min_cost) {
      min_cost = total_cost;
      min_point = next;
    }
  }

  return {min_cost, min_point};
}

  std::vector<PathPoint> PathCalculation::no_coloring_planning(std::vector<Cone> & cone_array,
                                                               common_lib::structures::Pose pose) {

    if (cone_array.size() < 4) {
      return {};
    }
    std::vector<std::unique_ptr<MidPoint>> midPoints;  // Use unique_ptr for memory management

    DT dt;

    for (auto cone : cone_array) {
      dt.insert(Point(cone.position.x, cone.position.y));
    }

    // Create midpoints using unique_ptr
    for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end();
         ++it) {
      Point p1 = it->first->vertex((it->second + 1) % 3)->point();
      Point p2 = it->first->vertex((it->second + 2) % 3)->point();
        
      auto midPoint = std::make_unique<MidPoint>(
          MidPoint{Point((p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2), {}});
      
      double dist = sqrt(pow(p1.x() - p2.x(), 2) +
                         pow(p1.y() - p2.y(), 2));
      if (dist>2.5){
      midPoints.push_back(std::move(midPoint));
      }
    }

    // Comparison function for priority queue using pointers
    auto cmp = [](const std::pair<double, MidPoint*>& a, const std::pair<double, MidPoint*>& b) {
      return a.first > b.first;
    };

    // Populate close points using pointers
    for (auto& p : midPoints) {
      std::priority_queue<std::pair<double, MidPoint*>, std::vector<std::pair<double, MidPoint*>>,
                          decltype(cmp)>
          pq(cmp);

      for (auto& q : midPoints) {
        if (p.get() == q.get()) continue;

        double dist = std::sqrt(std::pow(p->point.x() - q->point.x(), 2) +
                                std::pow(p->point.y() - q->point.y(), 2));
        pq.push({dist, q.get()});
      }

      // Take top 5 close points as pointers
      for (int i = 0; i < 6 && !pq.empty(); i++) {
        p->close_points.push_back(pq.top().second);
        pq.pop();
      }
    }

    // Find the first point closest to the current pose
    MidPoint* first = nullptr;
    double min_dist = 10000;
    for (auto& p : midPoints) {
      double dist =
          sqrt(pow(p->point.x() - pose.position.x, 2) + pow(p->point.y() - pose.position.y, 2));
      if (dist < min_dist) {
        min_dist = dist;
        first = p.get();
      }
    }

    // Project a point based on pose orientation
    double proj_distance = 1;  // 1 meter away from first point
    MidPoint projected = {Point(first->point.x() + cos(pose.orientation) * proj_distance,
                                first->point.y() + sin(pose.orientation) * proj_distance),
                          {}};

    // Find the second point closest to the projected point
    MidPoint* second = nullptr;
    min_dist = 10000;
    for (auto& p : midPoints) {
      double dist = sqrt(pow(p->point.x() - projected.point.x(), 2) +
                         pow(p->point.y() - projected.point.y(), 2));
      if (dist < min_dist) {
        min_dist = dist;
        second = p.get();
      }
    }

    // Rest of the path planning remains largely the same
    std::vector<MidPoint*> path;
    path.push_back(first);
    path.push_back(second);


    bool planning = true;
    int n_points = 0;
    while (planning) {
      double worst_cost = this->config_.max_cost_ * this->config_.search_depth_;

      std::pair<double, MidPoint*> best_result =
          dfs_cost(this->config_.search_depth_, path[path.size() - 2], path.back(), this->config_.max_cost_);

      if (best_result.first > worst_cost) {
        planning = false;
        break;
      }

      n_points++;
      if ((n_points > 60) || (best_result.second == path.back())) {
        planning = false;
      } else {
        path.push_back(best_result.second);
      }
    }

    // Convert path to final path points
    std::vector<PathPoint> final_path;
    for (MidPoint* p : path) {
      final_path.push_back(PathPoint(p->point.x(), p->point.y()));
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
    for (const Cone& cone : cones) {
      dt.insert(Point(cone.position.x, cone.position.y));
    }

    // Process valid triangulations and add positions to unordered_path
    for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end();
         ++it) {
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

  std::vector<PathPoint> PathCalculation::skidpad_path(std::vector<Cone> & cone_array,
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
      std::sort(cone_array.begin(), cone_array.end(), [&pose](Cone& a, Cone& b) {
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
      for (auto& point : hardcoded_path_) {
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
