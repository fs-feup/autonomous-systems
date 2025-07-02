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
                                                                        const MidPoint* previous,
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
    double distance = std::sqrt(std::pow(current->point.x() - next->point.x(), 2) +
                                std::pow(current->point.y() - next->point.y(), 2));

    // Angle calculation
    double angle_with_previous = std::atan2(current->point.y() - previous->point.y(),
                                            current->point.x() - previous->point.x());
    double angle_with_next =
        std::atan2(next->point.y() - current->point.y(), next->point.x() - current->point.x());

    // Normalize angle to be between 0 and π
    double angle = std::abs(angle_with_next - angle_with_previous);
    if (angle > M_PI) {
      angle = 2 * M_PI - angle;
    }

    // Local cost calculation
    double local_cost =
        std::pow(angle, this->config_.angle_exponent_) * this->config_.angle_gain_ +
        std::pow(distance, this->config_.distance_exponent_) * this->config_.distance_gain_;

    // Skip if local cost exceeds maximum allowed cost
    if (local_cost > maxcost) {
      continue;
    }

    // Recursive cost calculation
    auto [cost, selected_point] = dfs_cost(depth - 1, current, next, maxcost);

    // Total cost calculation
    double total_cost = local_cost + cost;

    // Update minimum cost and corresponding point
    if (total_cost < min_cost) {
      min_cost = total_cost;
      min_point = next;
    }
  }

  return {min_cost, min_point};
}

std::vector<PathPoint> PathCalculation::no_coloring_planning(const std::vector<Cone>& cone_array,
                                                             common_lib::structures::Pose pose) {
  std::vector<PathPoint> result;

  if (cone_array.size() >= 4) {
    // Create Delaunay triangulation and midpoints
    std::vector<std::unique_ptr<MidPoint>> mid_points =
        createMidPointsFromTriangulation(cone_array);

    // Create connections between midpoints
    establishMidPointConnections(mid_points);

    // Update anchor point if needed
    updateAnchorPoint(pose);

    // Find starting points for path
    auto [first, second] = findPathStartPoints(mid_points, anchor_point_);
    if (first != nullptr && second != nullptr) {
      // Generate path using DFS
      std::vector<MidPoint*> path = generatePath(first, second);

      // Convert to final path points
      result = convertToPathPoints(path);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to find valid starting points");
    }
  }

  return result;
}

std::vector<std::unique_ptr<PathCalculation::MidPoint>>
PathCalculation::createMidPointsFromTriangulation(const std::vector<Cone>& cone_array) {
  std::vector<std::unique_ptr<MidPoint>> mid_points;
  DT dt;

  // Insert cones into triangulation
  for (auto cone : cone_array) {
    auto _ = dt.insert(Point(cone.position.x, cone.position.y));
  }

  // Create midpoints
  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    Point p1 = it->first->vertex((it->second + 1) % 3)->point();
    Point p2 = it->first->vertex((it->second + 2) % 3)->point();

    auto midPoint = std::make_unique<MidPoint>(
        MidPoint{Point((p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2), {}});

    double dist = std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2));
    if (dist > config_.minimum_cone_distance_) {  // constrained by the rules (cones 3m apart)
      mid_points.push_back(std::move(midPoint));
    }
  }

  return mid_points;
}

void PathCalculation::establishMidPointConnections(
    const std::vector<std::unique_ptr<MidPoint>>& mid_points) {
  // Comparison function for priority queue
  auto cmp = [](const std::pair<double, MidPoint*>& distance_to_midpoint1,
                const std::pair<double, MidPoint*>& distance_to_midpoint2) {
    return distance_to_midpoint1.first > distance_to_midpoint2.first;
  };

  // Populate close points for each midpoint
  for (const auto& p : mid_points) {
    std::priority_queue<std::pair<double, MidPoint*>, std::vector<std::pair<double, MidPoint*>>,
                        decltype(cmp)>
        pq(cmp);

    for (const auto& q : mid_points) {
      if (p.get() == q.get()) {
        continue;
      }

      double dist = std::sqrt(std::pow(p->point.x() - q->point.x(), 2) +
                              std::pow(p->point.y() - q->point.y(), 2));
      pq.push({dist, q.get()});
    }

    // Take top 5 close points as pointers
    for (int i = 0; i < 6; i++) {
      if (pq.empty()) {
        break;
      }
      p->close_points.push_back(pq.top().second);
      pq.pop();
    }
  }
}

void PathCalculation::updateAnchorPoint(const common_lib::structures::Pose& pose) {
  if (!anchor_point_set_) {
    anchor_point_ = pose;
    anchor_point_set_ = true;
  }
}

std::pair<PathCalculation::MidPoint*, PathCalculation::MidPoint*>
PathCalculation::findPathStartPoints(const std::vector<std::unique_ptr<MidPoint>>& mid_points,
                                     const common_lib::structures::Pose& anchor_pose) {
  std::pair<MidPoint*, MidPoint*> result{nullptr, nullptr};

  auto cmp = [](const std::pair<double, MidPoint*>& distance_to_midpoint1,
                const std::pair<double, MidPoint*>& distance_to_midpoint2) {
    return distance_to_midpoint1.first > distance_to_midpoint2.first;
  };
  std::priority_queue<std::pair<double, MidPoint*>, std::vector<std::pair<double, MidPoint*>>,
                      decltype(cmp)>
      pq(cmp);

  // Create anchor midpoint (temporary, should be cleaned up)
  MidPoint* anchor_midpoint =
      new MidPoint{Point(anchor_pose.position.x, anchor_pose.position.y), {}};

  // Find midpoints that are in front of the car
  for (const auto& p : mid_points) {
    // Compute vector from the anchor point to the midpoint
    double dx = p->point.x() - anchor_midpoint->point.x();
    double dy = p->point.y() - anchor_midpoint->point.y();

    double car_direction_x = std::cos(anchor_pose.orientation);
    double car_direction_y = std::sin(anchor_pose.orientation);

    // Dot product checks whether the midpoint is in front of the car
    if ((dx * car_direction_x + dy * car_direction_y) <= 0.0) {
      continue;
    }

    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    pq.push({dist, p.get()});
  }

  // Get the closest midpoints in front of the car
  std::vector<MidPoint*> candidate_points;
  for (int i = 0; i < 6 && !pq.empty(); i++) {  // Fixed: was i <= 5
    candidate_points.push_back(pq.top().second);
    pq.pop();
  }

  // Set the anchor point's connections to these candidates
  anchor_midpoint->close_points = candidate_points;

  double best_cost = this->config_.max_cost_ * this->config_.search_depth_;

  // For each candidate first point, find the best second point using DFS
  for (MidPoint* first : anchor_midpoint->close_points) {
    auto [cost, second] =
        dfs_cost(this->config_.search_depth_, anchor_midpoint, first, this->config_.max_cost_);

    cost += std::pow(std::sqrt(std::pow(first->point.x() - anchor_midpoint->point.x(), 2) +
                               std::pow(first->point.y() - anchor_midpoint->point.y(), 2)),
                     this->config_.distance_exponent_) *
            this->config_.distance_gain_;
    if (cost < best_cost) {
      result.first = first;
      result.second = second;
      best_cost = cost;
    }
  }

  delete anchor_midpoint;

  return result;
}

std::vector<PathCalculation::MidPoint*> PathCalculation::generatePath(MidPoint* first,
                                                                      MidPoint* second) {
  std::vector<MidPoint*> path;
  path.push_back(first);
  path.push_back(second);

  int n_points = 0;
  while (true) {
    double worst_cost = this->config_.max_cost_ * this->config_.search_depth_;

    auto [best_cost, best_point] = dfs_cost(this->config_.search_depth_, path[path.size() - 2],
                                            path.back(), this->config_.max_cost_);

    if (best_cost > worst_cost) {
      break;
    }

    n_points++;
    if ((n_points > this->config_.max_points_) || (best_point == path.back())) {
      break;
    } else {
      path.push_back(best_point);
    }
  }

  return path;
}

std::vector<PathPoint> PathCalculation::convertToPathPoints(const std::vector<MidPoint*>& path) {
  std::vector<PathPoint> final_path;
  for (const MidPoint* p : path) {
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
      double dist = std::sqrt(std::pow(x_dist, 2) + std::pow(y_dist, 2));

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
      return std::sqrt(std::pow((a.position.x - pose.position.x), 2) +
                       std::pow((a.position.y - pose.position.y), 2)) <
             std::sqrt(std::pow((b.position.x - pose.position.x), 2) +
                       std::pow((b.position.y - pose.position.y), 2));
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
    double angle = std::atan(slope);

    // rotate all the points by the angle and add the origin point
    for (auto& point : hardcoded_path_) {
      double x = point.position.x;
      double y = point.position.y;
      point.position.x = x * std::cos(angle) - y * std::sin(angle) + middle_closest.position.x;
      point.position.y = x * std::sin(angle) + y * std::cos(angle) + middle_closest.position.y;
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
