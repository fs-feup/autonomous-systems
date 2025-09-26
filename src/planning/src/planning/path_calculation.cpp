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

std::pair<double, MidPoint*> PathCalculation::dfs_cost(int depth,
                                                                        const MidPoint* previous,
                                                                        MidPoint* current,
                                                                        double maxcost) {
  if (depth == 0) {
    return {0, current};  // Return current point if depth is 0
  }

  double min_cost = this->config_.max_cost_ * this->config_.search_depth_;
  MidPoint* min_point = current;  // Default to current point

  for (const auto& next : current->close_points) {
    // Avoid revisiting the previous point
    if (next.get() == previous) {
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
    auto [cost, selected_point] = dfs_cost(depth - 1, current, next.get(), maxcost);

    // Total cost calculation
    double total_cost = local_cost + cost;

    // Update minimum cost and corresponding point
    if (total_cost < min_cost) {
      min_cost = total_cost;
      min_point = next.get();
    }
  }

  return {min_cost, min_point};
}

std::vector<PathPoint> PathCalculation::no_coloring_planning(std::vector<Cone>& cone_array,
                                                             common_lib::structures::Pose pose) {
  std::vector<PathPoint> path_points;
  std::vector<Point> global_path;

  if (cone_array.size() < 4) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough cones to create a path.");
    path_points = {};
  } else {
    std::vector<std::shared_ptr<MidPoint>> midPoints;
    std::unordered_set<std::shared_ptr<Cone>> discarded_cones;

    // Generate midpoints between cone pairs using Delaunay triangulation
    create_mid_points(cone_array, midPoints, pose);

    // Map for quick access from Point to corresponding MidPoint
    std::unordered_map<Point, MidPoint*> point_to_midpoint;
    for (const auto& mp : midPoints) {
      point_to_midpoint[mp->point] = mp.get();
    }

    Point car_point(pose.position.x, pose.position.y);
    int max_points = config_.max_points_;
    path_update_counter_++;

    // Find the point in the current global path closest to the car
    int cutoff_index = -1;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < global_path_.size(); ++i) {
      double dist = CGAL::squared_distance(global_path_[i], car_point);
      if (dist < min_dist) {
        min_dist = dist;
        cutoff_index = static_cast<int>(i);
      }
    }

    if (cutoff_index == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid path points found.");
    }

    path_to_car.clear();
    // Retain part of the existing path leading to the car
    if (cutoff_index != -1 && cutoff_index > config_.lookback_points_) {
      (void)path_to_car.insert(path_to_car.end(), global_path_.begin(),
                               global_path_.begin() + cutoff_index - config_.lookback_points_);
    }

    std::unordered_set<MidPoint*> visited_midpoints;

    // Build initial path segment and extend it
    calculate_initial_path(global_path, midPoints, pose, point_to_midpoint, visited_midpoints,
                           discarded_cones);
    extend_path(global_path, midPoints, point_to_midpoint, visited_midpoints, discarded_cones,
                max_points);

    // Final processing: discard cones along the path and convert points
    for (const auto& point : global_path) {
      if (global_path.size() > 2) {
        discard_cones_along_path(global_path, midPoints, point_to_midpoint, discarded_cones);
      }
      (void)path_points.emplace_back(point.x(), point.y());
    }

    global_path_ = global_path;  // Update global path for next iteration
  }

  return path_points;
}

void PathCalculation::create_mid_points(std::vector<Cone>& cone_array,
                                        std::vector<std::shared_ptr<MidPoint>>& midPoints,
                                        const common_lib::structures::Pose& pose) {
  std::vector<Cone> active_cones;
  active_cones.reserve(cone_array.size());
                                          
  if(config_.use_sliding_window_) {
    for (const auto& cone : cone_array) {
      double dx = cone.position.x - pose.position.x;
      double dy = cone.position.y - pose.position.y;

      double sq_window_distance = config_.sliding_window_radius_ * config_.sliding_window_radius_; 

      if (dx * dx + dy * dy <= sq_window_distance) {
        active_cones.push_back(cone);
      }
    }

    if (active_cones.size() < 2) {
      RCLCPP_WARN(rclcpp::get_logger("planning"),"[Planning] Not enough cones in sliding window to compute midpoints");
      return;
    }
  }else {
    active_cones = std::move(cone_array);
  }
     
  this->midPoints.clear();
  this->triangulations.clear();
  DT dt;

  // Create shared_ptr for each cone for efficient sharing
  std::vector<std::shared_ptr<Cone>> cone_ptrs;
  cone_ptrs.reserve(active_cones.size());
  for (const auto& cone : active_cones) {
    cone_ptrs.push_back(std::make_shared<Cone>(cone));
  }

  // Insert all cone positions into the Delaunay triangulation
  for (const auto& cone_ptr : cone_ptrs) {
    (void)dt.insert(Point(cone_ptr->position.x, cone_ptr->position.y));
  }

  // Avoid duplicate midpoints for the same cone pair
  std::map<std::pair<int, int>, std::shared_ptr<MidPoint>> segment_to_midpoint;

  for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
    std::array<std::shared_ptr<MidPoint>, 3> mids;

    // Iterate over the 3 edges of the triangle
    for (int i = 0; i < 3; ++i) {
      Vertex_handle va = fit->vertex((i + 1) % 3);
      Vertex_handle vb = fit->vertex((i + 2) % 3);

      Point p1 = va->point();
      Point p2 = vb->point();

      // Using a map should be faster than searching every time! If an unordered map is used, it could be O(1).
      int id1 = ::find_cone(active_cones, p1.x(), p1.y());
      int id2 = ::find_cone(active_cones, p2.x(), p2.y());

      if (id1 == -1 || id2 == -1) {
        continue;
      }

      double sq_dist = CGAL::squared_distance(p1, p2);
      if ((sq_dist <= config_.minimum_cone_distance_ * config_.minimum_cone_distance_) ||
          (sq_dist >= config_.maximum_cone_distance_ * config_.maximum_cone_distance_)) {
        continue;
      }

      // Use ordered cone IDs to uniquely identify the segment
      auto key = std::minmax(id1, id2); // std::pair<min(a,b), max(a,b)>
      auto it = segment_to_midpoint.find(key);

      if (it != segment_to_midpoint.end()) {
        mids[i] = it->second;
      } else {
        auto midpoint = std::make_shared<MidPoint>(
            CGAL::midpoint(p1, p2), 
            std::vector<std::shared_ptr<MidPoint>>{}, 
            cone_ptrs[id1], 
            cone_ptrs[id2]
        );
        segment_to_midpoint[key] = midpoint;
        midPoints.push_back(midpoint);
        mids[i] = midpoint;
        //for the triangulations visualization
        triangulations.push_back({p1, p2});
      }
      
    }
    // Connect midpoints if they share the same triangle
    for (int i = 0; i < 3; ++i) {
      if (!mids[i]) {
        continue;
      }
      for (int j = 0; j < 3; ++j) {
        if (i == j || !mids[j]) {
          continue;
        }
        mids[i]->close_points.push_back(mids[j]);
      }
    }
  }

  for (const auto& p : midPoints) {
    this->midPoints.push_back(*p);
  }
}

void PathCalculation::calculate_initial_path(
    std::vector<Point>& path, const std::vector<std::shared_ptr<MidPoint>>& midPoints,
    const common_lib::structures::Pose& pose,
    const std::unordered_map<Point, MidPoint*>& point_to_midpoint,
    std::unordered_set<MidPoint*>& visited_midpoints, 
    std::unordered_set<std::shared_ptr<Cone>>& discarded_cones) {
  if (path_to_car.size() > 2) {
    RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Selecting initial path from %zu points.",
                 path_to_car.size());

    Point snapped_first = path_to_car[0];

    // Snap the first point to the nearest valid midpoint
    if (MidPoint* mp_first = find_nearest_point(snapped_first, point_to_midpoint); mp_first) {
      snapped_first = mp_first->point;
      (void)visited_midpoints.insert(mp_first);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for the first point.");
    }

    path.push_back(snapped_first);
    Point last_point = snapped_first;

    // Iterate over the remaining points from previous path
    for (std::size_t i = 1; i < path_to_car.size(); ++i) {
      Point current_point = path_to_car[i];
      MidPoint* current_mp = find_nearest_point(current_point, point_to_midpoint);
      if (current_mp) {
        current_point = current_mp->point;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("planning"),
                     "No valid midpoints found for the current point.");
      }

      double distance = CGAL::sqrt(CGAL::squared_distance(last_point, current_point));

      // Only add the point if it is far enough and not yet visited
      if (distance > config_.tolerance_ &&
          (current_mp == nullptr || visited_midpoints.count(current_mp) == 0)) {
        path.push_back(current_point);

        if (path.size() > 2) {
          discard_cones_along_path(path, midPoints, point_to_midpoint, discarded_cones);
        }

        last_point = current_point;
        if (current_mp) {
          (void)visited_midpoints.insert(current_mp);
        }
      }
    }
  } else {
    // Not enough path history, use pose to find start points
    update_anchor_point(pose);
    auto [first, second] = find_path_start_points(midPoints, anchor_pose_);
    if (first != nullptr && second != nullptr) {
      path.push_back(first->point);
      path.push_back(second->point);
      (void)visited_midpoints.insert(first);
      (void)visited_midpoints.insert(second);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to find valid starting points.");
    }
  }
}

void PathCalculation::extend_path(
    std::vector<Point>& path, const std::vector<std::shared_ptr<MidPoint>>& midPoints,
    const std::unordered_map<Point, MidPoint*>& point_to_midpoint,
    std::unordered_set<MidPoint*>& visited_midpoints, 
    std::unordered_set<std::shared_ptr<Cone>>& discarded_cones,
    int max_points) {
  int n_points = 0;
  // Define cost threshold for discarding poor path options
  double worst_cost = config_.max_cost_ * config_.search_depth_;

  while (true) {
    const auto& prev = path[path.size() - 2];
    const auto& last = path.back();

    const MidPoint* prev_mp = find_nearest_point(prev, point_to_midpoint);
    MidPoint* last_mp = find_nearest_point(last, point_to_midpoint);

    // Abort if midpoints can't be matched
    if (prev_mp == nullptr || last_mp == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for path extension.");
      break;
    }

    // Search for the best continuation from the current midpoint
    auto [best_cost, best_point] =
        dfs_cost(config_.search_depth_, prev_mp, last_mp, config_.max_cost_);

    // Stop if no good extension is found or point was already visited
    if (best_cost > worst_cost || best_point == nullptr ||
        visited_midpoints.count(best_point) > 0) {
      break;
    }

    path.push_back(best_point->point);
    (void)visited_midpoints.insert(best_point);
    n_points++;

    if (n_points > max_points) {
      break;
    }

    // Update midpoints validity and discard cones if needed
    if (path.size() > 2) {
      discard_cones_along_path(path, midPoints, point_to_midpoint, discarded_cones);
    }
  }
}

void PathCalculation::discard_cones_along_path(
    const std::vector<Point>& path, 
    const std::vector<std::shared_ptr<MidPoint>>& midPoints,
    const std::unordered_map<Point, MidPoint*>& point_to_midpoint,
    std::unordered_set<std::shared_ptr<Cone>>& discarded_cones) {
  const auto& last = path[path.size() - 2];
  const auto& current = path.back();

  MidPoint* last_mp = find_nearest_point(last, point_to_midpoint);
  const MidPoint* current_mp = find_nearest_point(current, point_to_midpoint);

  if (last_mp == nullptr || current_mp == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for discarding cones.");
    return;
  }

  // Identify a cone that was likely passed and should be discarded
  std::shared_ptr<Cone> discarded_cone = nullptr;

  if (last_mp->cone1 == current_mp->cone1 || last_mp->cone1 == current_mp->cone2) {
    if (last_mp->cone2 != current_mp->cone1 && last_mp->cone2 != current_mp->cone2) {
      discarded_cone = last_mp->cone2;
    }
  } else if (last_mp->cone2 == current_mp->cone1 || last_mp->cone2 == current_mp->cone2) {
    if (last_mp->cone1 != current_mp->cone1 && last_mp->cone1 != current_mp->cone2) {
      discarded_cone = last_mp->cone1;
    }
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("planning"), "No valid cones found for discarding.");
  }

  // Mark cone as discarded
  if (discarded_cone) {
    (void)discarded_cones.insert(discarded_cone);
  }

  // Invalidate midpoints that rely on discarded cones
  for (auto& mp : midPoints) {
    if (!mp->valid) {
      continue;
    }
    if (discarded_cones.count(mp->cone1) > 0 || discarded_cones.count(mp->cone2) > 0) {
      mp->valid = false;
    }
  }

  // Remove invalid neighbors from each midpoint's connections
  for (auto& mp : midPoints) {
    if (!mp->valid) {
      continue;
    }
    (void)mp->close_points.erase(
        std::remove_if(mp->close_points.begin(), mp->close_points.end(),
                       [](const std::shared_ptr<MidPoint>& neighbor) { return !neighbor->valid; }),
        mp->close_points.end());
  }
}

void PathCalculation::update_anchor_point(const common_lib::structures::Pose& pose) {
  if (!anchor_point_set_) {
    anchor_pose_ = pose;
    anchor_point_set_ = true;
  }
}

std::pair<MidPoint*, MidPoint*>
PathCalculation::find_path_start_points(const std::vector<std::shared_ptr<MidPoint>>& mid_points,
                                        const common_lib::structures::Pose& anchor_pose) {
  std::pair<MidPoint*, MidPoint*> result{nullptr, nullptr};

  auto cmp = [](const std::pair<double, MidPoint*>& cost1,
                const std::pair<double, MidPoint*>& cost2) {
    return cost1.first > cost2.first;
  };
  std::priority_queue<std::pair<double, MidPoint*>, std::vector<std::pair<double, MidPoint*>>,
                      decltype(cmp)>
      pq(cmp);

  MidPoint anchor_midpoint{
      Point(anchor_pose.position.x, anchor_pose.position.y), {}, nullptr, nullptr};

  // Find midpoints that are in front of the car
  for (const auto& p : mid_points) {
    double dx = p->point.x() - anchor_midpoint.point.x();
    double dy = p->point.y() - anchor_midpoint.point.y();
    double car_direction_x = std::cos(anchor_pose.orientation);
    double car_direction_y = std::sin(anchor_pose.orientation);
    if ((dx * car_direction_x + dy * car_direction_y) <= 0.0) {
      continue;
    }
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    double angle = std::atan2(p->point.y() - anchor_midpoint.point.y(),
                                            p->point.x() - anchor_midpoint.point.x());
    double cost =
        std::pow(angle, this->config_.angle_exponent_) * this->config_.angle_gain_ +
        std::pow(dist, this->config_.distance_exponent_) * this->config_.distance_gain_;
    pq.push({cost, p.get()});
  }

  // Get the closest midpoints in front of the car
  std::vector<MidPoint*> candidate_points;
  int count = 0;
  while (count < 8 && !pq.empty()) {
    candidate_points.push_back(pq.top().second);
    pq.pop();
    count++;
  }
  
  // Set the anchor point's connections to these candidates
  anchor_midpoint.close_points.clear();
  anchor_midpoint.close_points.reserve(candidate_points.size());

  for (auto* raw_ptr : candidate_points) {
    for (const auto& mp : mid_points) {
      if (mp.get() == raw_ptr) {
        anchor_midpoint.close_points.push_back(mp);
        break;
      }
    }
  }

  double best_cost = std::numeric_limits<double>::max();
  for (const auto& first : anchor_midpoint.close_points) {
    auto [cost, second] = dfs_cost(this->config_.search_depth_, &anchor_midpoint, first.get(),
                                   std::numeric_limits<double>::max());
    cost += std::pow(std::sqrt(std::pow(first->point.x() - anchor_midpoint.point.x(), 2) +
                               std::pow(first->point.y() - anchor_midpoint.point.y(), 2)),
                     this->config_.distance_exponent_) *
            this->config_.distance_gain_;
    if (cost < best_cost) {
      result.first = first.get();
      result.second = second;
      best_cost = cost;
    }
  }

  return result;
}

MidPoint* PathCalculation::find_nearest_point(
    const Point& target, const std::unordered_map<Point, MidPoint*>& map) {
  double min_dist_sq = config_.tolerance_ * config_.tolerance_;
  MidPoint* nearest = nullptr;

  for (const auto& [pt, mp] : map) {
    double dx = pt.x() - target.x();
    double dy = pt.y() - target.y();
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq <= min_dist_sq) {
      min_dist_sq = dist_sq;
      nearest = mp;
    }
  }

  return nearest;  // nullptr if none within tolerance
}

std::vector<PathPoint> PathCalculation::calculate_trackdrive(std::vector<Cone>& cone_array,
                                                             common_lib::structures::Pose pose) {
  vector<PathPoint> result = no_coloring_planning(cone_array, pose);

  // Check if we have enough points to form a loop
  if (result.size() < 3) {
    RCLCPP_WARN(rclcpp::get_logger("planning"), "Not enough points to create trackdrive loop");
    return result;
  }

  PathPoint first_point = result[0];
  PathPoint last_point = result.back();

  // Find the best point to close the loop by checking cost to connect back to first point
  double min_cost = std::numeric_limits<double>::max();
  int best_cutoff_index = result.size() - 1;  // Default to last point

  // Check each point in the path to find the best one to close the loop
  for (int i = 2; i < static_cast<int>(result.size()); ++i) {
    PathPoint current = result[i];
    PathPoint previous = result[i - 1];

    double dx = current.position.x - first_point.position.x;
    double dy = current.position.y - first_point.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double angle_with_previous = std::atan2(current.position.y - previous.position.y,
                                            current.position.x - previous.position.x);
    double angle_with_first = std::atan2(first_point.position.y - current.position.y,
                                         first_point.position.x - current.position.x);

    // Normalize angle to be between 0 and π
    double angle = std::abs(angle_with_first - angle_with_previous);
    if (angle > M_PI) {
      angle = 2 * M_PI - angle;
    }

    // Local cost calculation
    double cost =
        std::pow(angle, this->config_.angle_exponent_) * this->config_.angle_gain_ +
        std::pow(distance, this->config_.distance_exponent_) * this->config_.distance_gain_;

    if (cost < min_cost) {
      min_cost = cost;
      best_cutoff_index = i;
    }
  }

  // Trim the path to the best cutoff point
  result.erase(result.begin() + best_cutoff_index + 1, result.end());

  // Add X intermediate points between the last point and the first point
  int X = 4;  // You can adjust this value as needed
  if (X > 0 && result.size() > 0) {
    PathPoint last_point = result.back();
    PathPoint first_point = result[0];

    // Calculate the vector from last to first point
    float dx = first_point.position.x - last_point.position.x;
    float dy = first_point.position.y - last_point.position.y;

    // Add X evenly spaced intermediate points
    for (int i = 1; i <= X; ++i) {
      float t = static_cast<float>(i) / (X + 1);  // Interpolation parameter
      PathPoint intermediate;
      intermediate.position.x = last_point.position.x + t * dx;
      intermediate.position.y = last_point.position.y + t * dy;
      result.push_back(intermediate);
    }
  }

  // Close the loop by adding the first point again
  result.push_back(first_point);

  // Add overlap points (10 points or as many as available)
  int overlap_count = std::min(10, static_cast<int>(result.size()) -
                                       1);  // -1 to exclude the duplicate first point we just added
  for (int i = 1; i <= overlap_count; ++i) {
    result.push_back(result[i]);
  }

  return result;
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

std::vector<PathPoint> PathCalculation::get_global_path() const {
  std::vector<PathPoint> path_points;

  for (const auto& pt : path_to_car) {
    (void)path_points.emplace_back(pt.x(), pt.y());
  }

  return path_points;
}

std::vector<PathPoint> PathCalculation::skidpad_path(const std::vector<Cone>& cone_array,
                                                     common_lib::structures::Pose pose) {
  std::vector<PathPoint> result;

  // Load data from files only on first iteration
  if (!skidpad_data_loaded_) {
    const std::string file = "./src/planning/src/utils/skidpad.txt";
    const std::string conesfile = "./src/planning/src/utils/skidpadcones1.txt";

    // 1. Read reference cones from file
    std::ifstream cfile(conesfile);
    reference_cones_.clear();
    std::string line;
    while (std::getline(cfile, line)) {
      std::istringstream iss(line);
      double x = 0.0, y = 0.0, z = 0.0;
      if (iss >> x >> y >> z) {
        reference_cones_.emplace_back(x, y);
      } else {
        break;
      }
    }

    // 2. Read hardcoded path from file
    hardcoded_path_.clear();
    std::ifstream infile(file);
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      double x = 0.0, y = 0.0, v = 0.0;
      if (iss >> x >> y >> v) {
        hardcoded_path_.emplace_back(x, y, v);
      } else {
        break;
      }
    }

    skidpad_data_loaded_ = true;
  }

  // Check if we have enough cones for ICP
  if (reference_cones_.size() < static_cast<std::size_t>(config_.skidpad_minimum_cones_) ||
      cone_array.size() < static_cast<std::size_t>(config_.skidpad_minimum_cones_)) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough cones to perform ICP alignment.");
    return result;  // empty to indicate failure
  }

  // Perform ICP alignment on every iteration
  // 3. Convert cone_array (LiDAR) to source cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_source;
  cloud_source.reserve(cone_array.size());
  for (const auto& cone : cone_array) {
    cloud_source.emplace_back(cone.position.x, cone.position.y, 0.0);
  }

  // 4. Convert reference cones to target cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_target;
  cloud_target.reserve(reference_cones_.size());
  for (const auto& [x, y] : reference_cones_) {
    cloud_target.emplace_back(x, y, 0.0);
  }

  // 5. Run ICP alignment
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_source.makeShared());
  icp.setInputTarget(cloud_target.makeShared());
  icp.setMaxCorrespondenceDistance(config_.skidpad_tolerance_);
  icp.setMaximumIterations(std::numeric_limits<int>::max());
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-3);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  if (!icp.hasConverged()) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "ICP did not converge.");
    return result;  // empty to indicate failure
  }

  // 6. Apply transformation to hardcoded path (create a copy)
  result = hardcoded_path_;                                      // Copy the hardcoded path
  Eigen::Matrix4f icp_transform = icp.getFinalTransformation();  // LiDAR → MAP
  Eigen::Matrix4f map_to_lidar_transform = icp_transform.inverse();

  for (auto& point : result) {
    Eigen::Vector4f p(point.position.x, point.position.y, 0.0f, 1.0f);
    Eigen::Vector4f transformed = map_to_lidar_transform * p;
    point.position.x = transformed[0];
    point.position.y = transformed[1];
  }

  // Update predefined_path_ with the transformed path
  predefined_path_ = result;

  // Get the closest points to the car pose
  if (predefined_path_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Predefined path is empty.");
    return result;
  }

  double min_dist = std::numeric_limits<double>::max();
  size_t closest_index = 0;
  for (size_t i = 0; i < predefined_path_.size(); ++i) {
    double dx = predefined_path_[i].position.x - pose.position.x;
    double dy = predefined_path_[i].position.y - pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    } else if (dist > min_dist) {
      break;  // Stop searching if distance starts increasing
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("planning"),
              "predefined path size is %zu, closest index is %zu \n Hardcoded path size is %zu",
              predefined_path_.size(), closest_index, hardcoded_path_.size());
  // Remove all the points before the closest point not removing the closest point itself
  predefined_path_.erase(predefined_path_.begin(), predefined_path_.begin() + closest_index);
  hardcoded_path_.erase(hardcoded_path_.begin(), hardcoded_path_.begin() + closest_index);

  size_t path_size = predefined_path_.size();
  size_t count = 0;
  if (path_size >= 70) {
    count = 70;
  } else {
    count = path_size;
  }

  return std::vector<PathPoint>(predefined_path_.begin(), predefined_path_.begin() + count);
}