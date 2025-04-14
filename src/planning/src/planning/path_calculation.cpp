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

std::pair<double, PathCalculation::MidPoint*> PathCalculation::dfs_cost(int depth,
                                                                        MidPoint* previous,
                                                                        MidPoint* current,
                                                                        double maxcost) {
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
    double local_cost =
        pow(angle, this->config_.angle_exponent_) * this->config_.angle_gain_ +
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

std::vector<PathPoint> PathCalculation::no_coloring_planning(std::vector<Cone>& cone_array,
                                                             common_lib::structures::Pose pose) {
  if (cone_array.size() < 4) {
    return {};
  }
  std::vector<std::unique_ptr<MidPoint>> mid_points;  // Use unique_ptr for memory management

  DT dt;

  for (auto cone : cone_array) {
    dt.insert(Point(cone.position.x, cone.position.y));
  }

  // Create midpoints using unique_ptr
  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    Point p1 = it->first->vertex((it->second + 1) % 3)->point();
    Point p2 = it->first->vertex((it->second + 2) % 3)->point();

    auto midPoint = std::make_unique<MidPoint>(
        MidPoint{Point((p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2), {}});

    double dist = sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
    if (dist > 2.5) {
      mid_points.push_back(std::move(midPoint));
    }
  }

  // Comparison function for priority queue using pointers
  auto cmp = [](const std::pair<double, MidPoint*>& distance_to_midpoint1, const std::pair<double, MidPoint*>& distance_to_midpoint2) {
    return distance_to_midpoint1.first > distance_to_midpoint2.first;
  };

  // Populate close points using pointers
  // The idea of this portion of the code is to remove the overhead of checking the distances to all midpoints when calculating the path and just doing it once on the beggining and putting it on the close points vector
  // This is done by using a priority queue to sort the distances and then taking the top closest points
  for (auto& p : mid_points) {
    std::priority_queue<std::pair<double, MidPoint*>, std::vector<std::pair<double, MidPoint*>>,
                        decltype(cmp)>
        pq(cmp);

    for (auto& q : mid_points) {
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

  /* 
  Set the anchor point to the current pose if not set (first position of the car)
  This is done to calculate the path relative to a fixed point that we know for sure that is in the middle of the track
  This is done to avoid the path being calculated relative to the car position, which can be fatal if the car is not in the middle of the track
  The idea is to dinamically update this point in the future to only calculate the portion of the path that is needed 
  */
  if (!anchor_point_set_) {
    anchor_point_ = pose;
    anchor_point_set_ = true;
  }

  // Find the first point closest to the current pose
  MidPoint* first = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  for (auto& p : mid_points) {
    // Vector from car to the point
    double dx = p->point.x() - anchor_point_.position.x;
    double dy = p->point.y() - anchor_point_.position.y;

    double dist = sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      first = p.get();
    }
  }

  // Project a point based on pose orientation
  double proj_distance = 1;  // 1 meter away from first point
  if (first == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "No first point found");
    return {};
  }

  // Calculate the projected point from the first point
  MidPoint projected = {Point(first->point.x() + cos(anchor_point_.orientation) * proj_distance,
                              first->point.y() + sin(anchor_point_.orientation) * proj_distance),
                        {}};

  // Find the second point closest to the projected point that is in front of the car
  MidPoint* second = nullptr;
  min_dist = std::numeric_limits<double>::max();
  // Calculate the car's direction vector
  double car_direction_x = cos(anchor_point_.orientation);
  double car_direction_y = sin(anchor_point_.orientation);

  for (auto& p : mid_points) {
    if (p.get() == first) continue;

    // Compute vector from the first point to this candidate point.
    double dx = p->point.x() - first->point.x();
    double dy = p->point.y() - first->point.y();

    // Dot product checks whether the candidate is in front of the first point (relative to car's
    // orientation)
    double dot_product = dx * car_direction_x + dy * car_direction_y;

    if (dot_product <= 0.0) continue;  // This candidate is not in front; skip it.

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


  int n_points = 0;
  while (true) {
    double worst_cost = this->config_.max_cost_ * this->config_.search_depth_;

    std::pair<double, MidPoint*> best_result = dfs_cost(
        this->config_.search_depth_, path[path.size() - 2], path.back(), this->config_.max_cost_);

    if (best_result.first > worst_cost) {
      break;
    }

    n_points++;
    if ((n_points > this->config_.max_points_) || (best_result.second == path.back())) {
        break;
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
      RCLCPP_INFO(rclcpp::get_logger("planning"), "Cone not found in triangulations");
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

  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Unordered path size: %d",
               static_cast<int>(unordered_path.size()));

  return unordered_path;
}

std::vector<PathPoint> PathCalculation::skidpad_path(std::vector<Cone>& cone_array,
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
