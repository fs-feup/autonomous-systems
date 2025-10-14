#include "planning/path_calculation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
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
    std::unordered_set<Cone*> discarded_cones;

    // Generate midpoints between cone pairs using Delaunay triangulation
    create_mid_points(cone_array, midPoints);

    // Map for quick access from Point to corresponding MidPoint
    std::unordered_map<Point, MidPoint*, PointHash> point_to_midpoint;
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

    // Reset the path periodically to avoid long-term drift or degradation
    if (path_update_counter_ >= config_.reset_global_path_) {
      max_points = path_to_car.size() + config_.max_points_;
      path_to_car.clear();
      global_path_.clear();
      path_update_counter_ = 0;
      RCLCPP_INFO(rclcpp::get_logger("planning"), "Global path reset");
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
                                        std::vector<std::shared_ptr<MidPoint>>& midPoints) {
  this->midPoints.clear();
  DT dt;

  // Insert all cone positions into the Delaunay triangulation
  for (const auto& cone : cone_array) {
    (void)dt.insert(Point(cone.position.x, cone.position.y));
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

      int id1 = ::find_cone(cone_array, p1.x(), p1.y());
      int id2 = ::find_cone(cone_array, p2.x(), p2.y());

      if (id1 == -1 || id2 == -1) {
        continue;
      }

      double sq_dist = CGAL::squared_distance(p1, p2);
      if ((sq_dist <= config_.minimum_cone_distance_ * config_.minimum_cone_distance_) ||
          (sq_dist >= config_.maximum_cone_distance_ * config_.maximum_cone_distance_)) {
        continue;
      }

      // Use ordered cone IDs to uniquely identify the segment
      auto key = std::minmax(id1, id2);
      auto it = segment_to_midpoint.find(key);

      if (it != segment_to_midpoint.end()) {
        mids[i] = it->second;
      } else {
        auto midpoint = std::make_shared<MidPoint>(
            MidPoint{CGAL::midpoint(p1, p2), {}, &cone_array[id1], &cone_array[id2]});
        segment_to_midpoint[key] = midpoint;
        midPoints.push_back(midpoint);
        mids[i] = midpoint;
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
    const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
    std::unordered_set<MidPoint*>& visited_midpoints, std::unordered_set<Cone*>& discarded_cones) {
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
    const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
    std::unordered_set<MidPoint*>& visited_midpoints, std::unordered_set<Cone*>& discarded_cones,
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
    const std::vector<Point>& path, const std::vector<std::shared_ptr<MidPoint>>& midPoints,
    const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
    std::unordered_set<Cone*>& discarded_cones) {
  const auto& last = path[path.size() - 2];
  const auto& current = path.back();

  MidPoint* last_mp = find_nearest_point(last, point_to_midpoint);
  const MidPoint* current_mp = find_nearest_point(current, point_to_midpoint);

  if (last_mp == nullptr || current_mp == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for discarding cones.");
    return;
  }

  // Identify a cone that was likely passed and should be discarded
  Cone* discarded_cone = nullptr;

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
  (void)discarded_cones.insert(discarded_cone);

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

std::pair<PathCalculation::MidPoint*, PathCalculation::MidPoint*>
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

PathCalculation::MidPoint* PathCalculation::find_nearest_point(
    const Point& target, const std::unordered_map<Point, MidPoint*, PointHash>& map) {
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

using namespace std;

// ===================== Path Calculation (Core) =====================

vector<PathPoint> PathCalculation::calculate_path(vector<Cone>& cone_array) {

  if (cone_array.size() < 4) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough cones to create a path.");
    return {};
  }

  clear_path_state();

  // Determine if we should regenerate all midpoints (path reset)
  bool should_reset = config_.use_reset_path_ && reset_path_counter_ >= config_.reset_path_;

  // Generate midpoints using the generator
  midpoints_ = midpoint_generator_.generate_midpoints(cone_array, should_reset);

  // Map for quick access from Point to corresponding Midpoint
  for (const auto& mp : midpoints_) {
    point_to_midpoint_[mp->point] = mp;
  }

  Point car_point(vehicle_pose_.position.x, vehicle_pose_.position.y);

  // Find the point in the past_path_ closest to the car
  int cutoff_index = -1;
  double min_dist = numeric_limits<double>::max();
  for (size_t i = 0; i < past_path_.size(); ++i) {
    double dist = CGAL::squared_distance(past_path_[i], car_point);
    if (dist < min_dist) {
      min_dist = dist;
      cutoff_index = static_cast<int>(i);
    }
  }

  if (cutoff_index == -1) {
    RCLCPP_WARN(rclcpp::get_logger("planning"), "No valid path points found near the car.");
  }

  path_to_car_.clear();
  // Retain part of the existing path leading to the car
  if (cutoff_index != -1 && cutoff_index > config_.lookback_points_) {
    (void)path_to_car_.insert(path_to_car_.end(), past_path_.begin(),
                        past_path_.begin() + cutoff_index - config_.lookback_points_);
  }

  int max_points = reset_path(should_reset);

  // Build initial path segment
  if (path_to_car_.size() <= 2) {
    initialize_path_from_initial_pose();
  } else {
    update_path_from_past_path();
  }

  extend_path(max_points);

  past_path_ = current_path_;  // Update the path for next iteration

  return get_path_points_from_points(current_path_);
}

vector<PathPoint> PathCalculation::calculate_trackdrive(vector<Cone>& cone_array) {
  vector<PathPoint> result = calculate_path(cone_array);

  // Check if we have enough points to form a loop
  if (result.size() < 3) {
    RCLCPP_WARN(rclcpp::get_logger("planning"), "Not enough points to create trackdrive loop");
    return result;
  }

  // Find the best point to close the loop
  int best_cutoff_index = find_best_loop_closure(result);

  // Trim the path to the best cutoff point
  (void)result.erase(result.begin() + best_cutoff_index + 1, result.end());

  // Add interpolated points between the last point and the first point
  if (!result.empty()) {
    const PathPoint& last_point = result.back();
    const PathPoint& first_point = result.front();

    vector<PathPoint> interpolated = add_interpolated_points(last_point, first_point, 4);
    (void)result.insert(result.end(), interpolated.begin(), interpolated.end());
  }

  // Close the loop by adding the first point again
  result.push_back(result[0]);

  // Add overlap points (10 points or as many as available)
  int overlap_count = min(10, static_cast<int>(result.size()) - 1);
  for (int i = 1; i <= overlap_count; ++i) {
    result.push_back(result[i]);
  }

  return result;
}

// ===================== State Management =====================

void PathCalculation::clear_path_state() {
  current_path_.clear();
  point_to_midpoint_.clear();
  visited_midpoints_.clear();
  discarded_cones_.clear();
}

int PathCalculation::reset_path(bool should_reset) {
  int max_points = config_.max_points_;
  reset_path_counter_++;

  if (should_reset) {
    max_points = static_cast<int>(path_to_car_.size()) + config_.max_points_;
    path_to_car_.clear();
    past_path_.clear();
    reset_path_counter_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("planning"), "Global path reset");
  }

  return max_points;
}

void PathCalculation::set_vehicle_pose(const common_lib::structures::Pose& vehicle_pose) {
  midpoint_generator_.set_vehicle_pose(vehicle_pose);
  vehicle_pose_ = vehicle_pose;
}

// ===================== Path Initialization =====================

void PathCalculation::initialize_path_from_initial_pose() {
  if (!initial_pose_set_) {
    initial_pose_ = vehicle_pose_;
    initial_pose_set_ = true;
  }

  const auto [first_point, second_point] = select_starting_midpoints();

  if (first_point && second_point) {
    current_path_.push_back(first_point->point);
    current_path_.push_back(second_point->point);
    (void)visited_midpoints_.insert(first_point);
    (void)visited_midpoints_.insert(second_point);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to find valid starting points.");
  }
}

void PathCalculation::update_path_from_past_path() {
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Selecting initial path from %zu points.",
               path_to_car_.size());

  Point last_added_point;
  bool first_point_added = false;

  for (const Point& path_to_car_point : path_to_car_) {
    // By default, use the original point
    Point candidate_point = path_to_car_point;

    // Snap to nearest valid midpoint
    shared_ptr<Midpoint> candidate_midpoint = find_nearest_midpoint(path_to_car_point);

    // If found a valid midpoint, use it
    if (candidate_midpoint) {
      candidate_point = candidate_midpoint->point;
    }

    // Check if already visited
    if (candidate_midpoint && visited_midpoints_.count(candidate_midpoint) > 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Skipping point: Already visited.");
      continue;
    }

    // For non-first points, check distance from last added point
    if (first_point_added) {
      double distance = sqrt(CGAL::squared_distance(last_added_point, candidate_point));
      if (distance <= config_.tolerance_) {
        RCLCPP_DEBUG(rclcpp::get_logger("planning"),
                     "Skipping point: Too close to last added point.");
        continue;
      }
    }

    // If the candidate_midpoint is valid, add it to visited set
    if (candidate_midpoint) {
      (void)visited_midpoints_.insert(candidate_midpoint);
    }

    current_path_.push_back(candidate_point);
    last_added_point = candidate_point;
    first_point_added = true;
  }
}

// ===================== Path Extension =====================

void PathCalculation::extend_path(int max_points) {
  int n_points = 0;
  // Define cost threshold for discarding poor path options
  const double worst_cost = config_.max_cost_ * config_.search_depth_;

  while (true) {
    if (current_path_.size() < 2) {
      break;
    }

    const Point& prev = current_path_[current_path_.size() - 2];
    const Point& last = current_path_.back();

    shared_ptr<Midpoint> prev_mp = find_nearest_midpoint(prev);
    shared_ptr<Midpoint> last_mp = find_nearest_midpoint(last);

    // Abort if midpoints can't be matched
    if (!prev_mp || !last_mp) {
      RCLCPP_WARN(rclcpp::get_logger("planning"),
                  "No valid midpoints found for path extension.");
      break;
    }

    // Search for the best continuation from the current midpoint
    auto [best_cost, best_point] = find_best_next_midpoint(
        config_.search_depth_, prev_mp, last_mp, config_.max_cost_);

    // Stop if no good extension is found or point was already visited
    if (best_cost > worst_cost || !best_point || visited_midpoints_.count(best_point) > 0) {
      break;
    }

    current_path_.push_back(best_point->point);
    (void)visited_midpoints_.insert(best_point);
    n_points++;

    if (n_points >= max_points) {
      break;
    }

    // Update midpoints validity and discard cones if needed
    if (current_path_.size() > 2) {
      discard_cones_along_path();
    }
  }
}

// ================================ Path Search ================================

pair<shared_ptr<Midpoint>, shared_ptr<Midpoint>> PathCalculation::select_starting_midpoints() {
  pair<shared_ptr<Midpoint>, shared_ptr<Midpoint>> result{nullptr, nullptr};

  Midpoint anchor_pose_midpoint{
      Point(initial_pose_.position.x, initial_pose_.position.y), nullptr, nullptr};

  // Get candidate midpoints
  anchor_pose_midpoint.close_points = select_candidate_midpoints(anchor_pose_midpoint, 8);

  double best_cost = numeric_limits<double>::max();
  for (const auto& first : anchor_pose_midpoint.close_points) {
    shared_ptr<Midpoint> anchor_mp = make_shared<Midpoint>(anchor_pose_midpoint);

    auto [cost, midpoint] = find_best_next_midpoint(
        config_.search_depth_, anchor_mp, first, numeric_limits<double>::max());

    double dx = first->point.x() - anchor_pose_midpoint.point.x();
    double dy = first->point.y() - anchor_pose_midpoint.point.y();
    double initial_distance =
        pow(sqrt(dx * dx + dy * dy), config_.distance_exponent_) * config_.distance_gain_;

    cost += initial_distance;

    if (cost < best_cost) {
      result.first = first;
      result.second = midpoint;
      best_cost = cost;
    }
  }

  return result;
}

vector<shared_ptr<Midpoint>> PathCalculation::select_candidate_midpoints(
    const Midpoint& anchor_pose, int num_candidates) const {
  auto cmp = [](const pair<double, shared_ptr<Midpoint>>& cost1,
                const pair<double, shared_ptr<Midpoint>>& cost2) {
    return cost1.first > cost2.first;
  };

  priority_queue<pair<double, shared_ptr<Midpoint>>,
                 vector<pair<double, shared_ptr<Midpoint>>>, decltype(cmp)>
      pq(cmp);

  double car_direction_x = cos(initial_pose_.orientation);
  double car_direction_y = sin(initial_pose_.orientation);

  // Find midpoints that are in front of the car
  for (const auto& mp : midpoints_) {
    double dx = mp->point.x() - anchor_pose.point.x();
    double dy = mp->point.y() - anchor_pose.point.y();

    // Skip points behind the car
    if ((dx * car_direction_x + dy * car_direction_y) <= 0.0) {
      continue;
    }

    double dist = sqrt(dx * dx + dy * dy);
    double angle = atan2(dy, dx);
    double cost = pow(angle, config_.angle_exponent_) * config_.angle_gain_ +
                  pow(dist, config_.distance_exponent_) * config_.distance_gain_;
    pq.push({cost, mp});
  }

  // Get the closest midpoints in front of the car
  vector<shared_ptr<Midpoint>> candidate_points;
  int count = 0;
  while (count < num_candidates && !pq.empty()) {
    candidate_points.push_back(pq.top().second);
    pq.pop();
    count++;
  }

  return candidate_points;
}

pair<double, shared_ptr<Midpoint>> PathCalculation::find_best_next_midpoint(
    int depth, const shared_ptr<Midpoint>& previous, const shared_ptr<Midpoint>& current,
    double max_cost) const {
  if (depth == 0) {
    return {0.0, current};
  }

  double min_cost = config_.max_cost_ * config_.search_depth_;
  shared_ptr<Midpoint> min_point = current;

  for (const auto& next : current->close_points) {
    // Avoid revisiting the previous point
    if (next == previous) {
      continue;
    }

    double local_cost = calculate_midpoint_cost(previous, current, next);

    // Skip if local cost exceeds maximum allowed cost
    if (local_cost > max_cost) {
      continue;
    }

    // Recursive cost calculation
    pair<double, shared_ptr<Midpoint>> recursive_result =
        find_best_next_midpoint(depth - 1, current, next, max_cost);

    double total_cost = local_cost + recursive_result.first;

    // Update minimum cost and corresponding point
    if (total_cost < min_cost) {
      min_cost = total_cost;
      min_point = next;
    }
  }

  return {min_cost, min_point};
}

double PathCalculation::calculate_midpoint_cost(const shared_ptr<Midpoint>& previous,
                                                const shared_ptr<Midpoint>& current,
                                                const shared_ptr<Midpoint>& next) const {
  double dx_dist = current->point.x() - next->point.x();
  double dy_dist = current->point.y() - next->point.y();
  double distance = sqrt(dx_dist * dx_dist + dy_dist * dy_dist);

  // Angle calculation
  double angle_with_previous =
      atan2(current->point.y() - previous->point.y(),
            current->point.x() - previous->point.x());
  double angle_with_next = atan2(next->point.y() - current->point.y(),
                                  next->point.x() - current->point.x());

  // Normalize angle to be between 0 and π
  double angle = abs(angle_with_next - angle_with_previous);
  if (angle > M_PI) {
    angle = 2 * M_PI - angle;
  }

  // Local cost calculation
  return pow(angle, config_.angle_exponent_) * config_.angle_gain_ +
         pow(distance, config_.distance_exponent_) * config_.distance_gain_;
}

// ===================== Cone Discarding =====================

void PathCalculation::discard_cones_along_path() {
  if (current_path_.size() < 2) {
    return;
  }

  const Point& last = current_path_[current_path_.size() - 2];
  const Point& current = current_path_.back();

  shared_ptr<Midpoint> last_mp = find_nearest_midpoint(last);
  shared_ptr<Midpoint> current_midpoint = find_nearest_midpoint(current);

  if (!last_mp || !current_midpoint) {
    RCLCPP_WARN(rclcpp::get_logger("planning"),
                "No valid midpoints found for discarding cones.");
    return;
  }

  // Discard a cone that was likely passed and should be discarded
  discard_cone(last_mp, current_midpoint);

  // Invalidate midpoints that rely on discarded cones
  invalidate_midpoints_with_discarded_cones();

  // Remove invalid neighbors from each midpoint's connections
  remove_invalid_neighbors();
}

void PathCalculation::discard_cone(const shared_ptr<Midpoint>& last_mp,
                                    const shared_ptr<Midpoint>& current_mp) {
  if (last_mp->cone1 == current_mp->cone1 || last_mp->cone1 == current_mp->cone2) {
    if (last_mp->cone2 != current_mp->cone1 && last_mp->cone2 != current_mp->cone2) {
      (void)discarded_cones_.insert(last_mp->cone2);
    }
  } else if (last_mp->cone2 == current_mp->cone1 || last_mp->cone2 == current_mp->cone2) {
    if (last_mp->cone1 != current_mp->cone1 && last_mp->cone1 != current_mp->cone2) {
      (void)discarded_cones_.insert(last_mp->cone1);
    }
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("planning"), "No valid cones found for discarding.");
  }
}

void PathCalculation::invalidate_midpoints_with_discarded_cones() {
  for (const auto& mp : midpoints_) {
    if (!mp->valid) {
      continue;
    }
    if (discarded_cones_.count(mp->cone1) > 0 || discarded_cones_.count(mp->cone2) > 0) {
      mp->valid = false;
    }
  }
}

void PathCalculation::remove_invalid_neighbors() {
  for (const auto& mp : midpoints_) {
    if (!mp->valid) {
      continue;
    }
    (void)mp->close_points.erase(
        remove_if(mp->close_points.begin(), mp->close_points.end(),
                  [](const shared_ptr<Midpoint>& neighbor) { return !neighbor->valid; }),
        mp->close_points.end());
  }
}

// ===================== Utility Methods =====================

shared_ptr<Midpoint> PathCalculation::find_nearest_midpoint(const Point& target) const {
  double min_dist_sq = config_.tolerance_ * config_.tolerance_;
  shared_ptr<Midpoint> nearest = nullptr;

  for (const auto& [pt, mp] : point_to_midpoint_) {
    double dx = pt.x() - target.x();
    double dy = pt.y() - target.y();
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq <= min_dist_sq) {
      min_dist_sq = dist_sq;
      nearest = mp;
    }
  }

  return nearest;
}

int PathCalculation::find_best_loop_closure(const vector<PathPoint>& path) const {
  if (path.size() < 3) {
    return path.size() - 1;
  }

  const PathPoint& first_point = path[0];
  double min_cost = numeric_limits<double>::max();
  int best_cutoff_index = static_cast<int>(path.size() - 1);

  // Check each point in the path to find the best one to close the loop
  for (int i = 2; i < static_cast<int>(path.size()); ++i) {
    const PathPoint& current = path[i];
    const PathPoint& previous = path[i - 1];

    double dx = current.position.x - first_point.position.x;
    double dy = current.position.y - first_point.position.y;
    double distance = sqrt(dx * dx + dy * dy);

    double angle_with_previous = atan2(current.position.y - previous.position.y,
                                       current.position.x - previous.position.x);
    double angle_with_first = atan2(first_point.position.y - current.position.y,
                                    first_point.position.x - current.position.x);

    // Normalize angle to be between 0 and π
    double angle = abs(angle_with_first - angle_with_previous);
    if (angle > M_PI) {
      angle = 2 * M_PI - angle;
    }

    // Local cost calculation
    double cost = pow(angle, config_.angle_exponent_) * config_.angle_gain_ +
                  pow(distance, config_.distance_exponent_) * config_.distance_gain_;

    if (cost < min_cost) {
      min_cost = cost;
      best_cutoff_index = i;
    }
  }

  return best_cutoff_index;
}

vector<PathPoint> PathCalculation::add_interpolated_points(const PathPoint& start,
                                                            const PathPoint& end,
                                                            int num_points) const {
  vector<PathPoint> interpolated;
  interpolated.reserve(num_points);

  if (num_points <= 0) {
    return interpolated;
  }

  float dx = end.position.x - start.position.x;
  float dy = end.position.y - start.position.y;

  for (int i = 1; i <= num_points; ++i) {
    float t = static_cast<float>(i) / (num_points + 1);
    PathPoint intermediate;
    intermediate.position.x = start.position.x + t * dx;
    intermediate.position.y = start.position.y + t * dy;
    interpolated.push_back(intermediate);
  }

  return interpolated;
}

std::vector<PathPoint> PathCalculation::get_path_points_from_points(const std::vector<Point>& points) const{
  vector<PathPoint> path_points;
  path_points.reserve(points.size());

  for(Point pt : points){
    (void)path_points.emplace_back(pt.x(), pt.y());
  }

  return path_points;
}
// ===================== Accessor Methods =====================

vector<PathPoint> PathCalculation::get_path_to_car() const {
  return get_path_points_from_points(path_to_car_);
}

const vector<pair<Point, Point>>& PathCalculation::get_triangulations() const {
  return midpoint_generator_.get_triangulations();
}