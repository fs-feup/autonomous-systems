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

double PathCalculation::calculate_midpoint_cost(
    const std::shared_ptr<Midpoint>& previous,     
    const std::shared_ptr<Midpoint>& current,       
    const std::shared_ptr<Midpoint>& next) {        
  
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
  return std::pow(angle, this->config_.angle_exponent_) * this->config_.angle_gain_ +
         std::pow(distance, this->config_.distance_exponent_) * this->config_.distance_gain_;
}

std::pair<double, std::shared_ptr<Midpoint>> PathCalculation::find_best_next_midpoint(  
    int depth,
    const std::shared_ptr<Midpoint>& previous,     
    const std::shared_ptr<Midpoint>& current,       
    double maxcost) {

  if (depth == 0) {
    return {0, current};  // Return current point if depth is 0
  }

  double min_cost = this->config_.max_cost_ * this->config_.search_depth_;
  std::shared_ptr<Midpoint> min_point = current;  // Default to current point

  for (const auto& next : current->close_points) {
    // Avoid revisiting the previous point
    if (next == previous) {
      continue;
    }

    double local_cost = calculate_midpoint_cost(previous, current, next);

    // Skip if local cost exceeds maximum allowed cost
    if (local_cost > maxcost) {
      continue;
    }

    // Recursive cost calculation
    auto [cost, selected_point] = find_best_next_midpoint(depth - 1, current, next, maxcost);

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

void PathCalculation::clear_path_state() {
  current_path_.clear();
  point_to_midpoint_.clear();
  visited_midpoints_.clear();
  discarded_cones_.clear();
}

std::vector<PathPoint> PathCalculation::calculate_path(std::vector<Cone>& cone_array) {
  std::vector<PathPoint> path_points;

  if (cone_array.size() < 4) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough cones to create a path.");
    return {};
  }

  clear_path_state();

  // Determine if we should regenerate all midpoints (path reset)
  bool should_reset = (config_.use_reset_path_ && 
                       reset_path_counter_ >= config_.reset_global_path_);
  
  // Generate midpoints using the generator
  midpoints_ = midpoint_generator_.generate_midpoints(cone_array, should_reset);

  // Map for quick access from Point to corresponding Midpoint
  for (const auto& mp : midpoints_) {
    point_to_midpoint_[mp->point] = mp;
  }

  Point car_point(vehicle_pose_.position.x, vehicle_pose_.position.y);

  // Find the point in the past_path_ closest to the car
  int cutoff_index = -1;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < past_path_.size(); ++i) {
    double dist = CGAL::squared_distance(past_path_[i], car_point);
    if (dist < min_dist) {
      min_dist = dist;
      cutoff_index = static_cast<int>(i);
    }
  }

  if (cutoff_index == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid path points found near the car.");
  }

  path_to_car.clear();
  // Retain part of the existing path leading to the car
  if (cutoff_index != -1 && cutoff_index > config_.lookback_points_) {
    (void)path_to_car.insert(path_to_car.end(), past_path_.begin(),
                             past_path_.begin() + cutoff_index - config_.lookback_points_);
  }

  int max_points = reset_path(should_reset);

  // Build initial path segment and it
  calculate_initial_path();
  extend_path(max_points);

  // Final processing: discard cones along the path and convert points
  for (const auto& point : current_path_) {
    // if (current_path_.size() > 2) {
    //   discard_cones_along_path();
    // }
    (void)path_points.emplace_back(point.x(), point.y());
  }

  past_path_ = current_path_;  // Update the path for next iteration

  return path_points;
}

int PathCalculation::reset_path(bool should_reset) {
  int max_points = config_.max_points_;
  reset_path_counter_++;

  if (should_reset) {
    max_points = path_to_car.size() + config_.max_points_;
    path_to_car.clear();
    past_path_.clear();
    reset_path_counter_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("planning"), "Global path reset");
  }

  return max_points;
}

void PathCalculation::update_path_from_past_path() {
  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Selecting initial path from %zu points.",
               path_to_car.size());

  Point last_added_point;
  bool first_point_added = false;

  for(const Point& path_to_car_point : path_to_car) {
    //By default, use the original point
    Point candidate_point = path_to_car_point;

    //Snap to nearest valid midpoint
    std::shared_ptr<Midpoint> candidate_midpoint = find_nearest_midpoint(path_to_car_point);

    //If found a valid midpoint, use it
    if(candidate_midpoint){
      candidate_point = candidate_midpoint->point;
    }

    //Check if already visited
    if(candidate_midpoint && visited_midpoints_.count(candidate_midpoint) > 0){
      RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Skipping point: Already visited.");
      continue;
    }

    //For non-first points, check distance from last added point
    if(first_point_added){
      double distance = CGAL::sqrt(CGAL::squared_distance(last_added_point, candidate_point));
      if(distance <= config_.tolerance_){
        RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Skipping point: Too close to last added point.");
        continue;
      }
    }

    //If the candidate_midpoint is valid add it to visited set
    if(candidate_midpoint){
      (void)visited_midpoints_.insert(candidate_midpoint);
    }
    current_path_.push_back(candidate_point);
    last_added_point = candidate_point;
    first_point_added = true;

  }
}

void PathCalculation::initialize_path_from_anchor() {
  if (!anchor_pose_set_) {
    initial_pose_ = vehicle_pose_;
    anchor_pose_set_ = true;
  }
  auto [first, second] = select_starting_midpoints();
  if (first != nullptr && second != nullptr) {
    current_path_.push_back(first->point);
    current_path_.push_back(second->point);
    (void)visited_midpoints_.insert(first);
    (void)visited_midpoints_.insert(second);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to find valid starting points.");
  }
}

void PathCalculation::calculate_initial_path() {
  if (path_to_car.size() <= 2) {
    initialize_path_from_anchor();
  } else {
    update_path_from_past_path();
  }
}

void PathCalculation::extend_path(int max_points) {
  int n_points = 0;
  // Define cost threshold for discarding poor path options
  double worst_cost = config_.max_cost_ * config_.search_depth_;

  while (true) {
    const auto& prev = current_path_[current_path_.size() - 2];
    const auto& last = current_path_.back();

    std::shared_ptr<Midpoint> prev_mp = find_nearest_midpoint(prev);     
    std::shared_ptr<Midpoint> last_mp = find_nearest_midpoint(last);     

    // Abort if midpoints can't be matched
    if (prev_mp == nullptr || last_mp == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for path extension.");
      break;
    }

    // Search for the best continuation from the current midpoint
    auto [best_cost, best_point] =
        find_best_next_midpoint(config_.search_depth_, prev_mp, last_mp, config_.max_cost_);

    // Stop if no good extension is found or point was already visited
    if (best_cost > worst_cost || best_point == nullptr ||
        visited_midpoints_.count(best_point) > 0) {
      break;
    }

    current_path_.push_back(best_point->point);
    (void)visited_midpoints_.insert(best_point);
    n_points++;

    if (n_points > max_points) {
      break;
    }

    // Update midpoints validity and discard cones if needed
    if (current_path_.size() > 2) {
      discard_cones_along_path();
    }
  }
}

void PathCalculation::discard_cone(
    const std::shared_ptr<Midpoint>& last_mp,        
    const std::shared_ptr<Midpoint>& current_mp) {   
  
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
        std::remove_if(mp->close_points.begin(), mp->close_points.end(),
                       [](const std::shared_ptr<Midpoint>& neighbor) {  
                         return !neighbor->valid;
                       }),
        mp->close_points.end());
  }
}

void PathCalculation::discard_cones_along_path() {
  const auto& last = current_path_[current_path_.size() - 2];
  const auto& current = current_path_.back();

  std::shared_ptr<Midpoint> last_mp = find_nearest_midpoint(last);              
  std::shared_ptr<Midpoint> current_midpoint = find_nearest_midpoint(current);  

  if (last_mp == nullptr || current_midpoint == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for discarding cones.");
    return;
  }

  // Discard a cone that was likely passed and should be discarded
  discard_cone(last_mp, current_midpoint);

  // Invalidate midpoints that rely on discarded cones
  invalidate_midpoints_with_discarded_cones();

  // Remove invalid neighbors from each midpoint's connections
  remove_invalid_neighbors();
}

void PathCalculation::update_vehicle_pose(const common_lib::structures::Pose& vehicle_pose) {
  midpoint_generator_.set_vehicle_pose(vehicle_pose);
  vehicle_pose_ = vehicle_pose;
}

std::vector<std::shared_ptr<Midpoint>> PathCalculation::select_candidate_midpoints(  
    const Midpoint& anchor_pose,                                                     
    int num_candidates) {
  
  auto cmp = [](const std::pair<double, std::shared_ptr<Midpoint>>& cost1,  
                const std::pair<double, std::shared_ptr<Midpoint>>& cost2) { 
    return cost1.first > cost2.first;
  };
  std::priority_queue<std::pair<double, std::shared_ptr<Midpoint>>,         
                      std::vector<std::pair<double, std::shared_ptr<Midpoint>>>,  
                      decltype(cmp)> pq(cmp);

  // Find midpoints that are in front of the car
  for (const auto& mp : midpoints_) {
    double dx = mp->point.x() - anchor_pose.point.x();
    double dy = mp->point.y() - anchor_pose.point.y();
    double car_direction_x = std::cos(initial_pose_.orientation);
    double car_direction_y = std::sin(initial_pose_.orientation);
    
    if ((dx * car_direction_x + dy * car_direction_y) <= 0.0) {
      continue;
    }
    
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    double angle = std::atan2(mp->point.y() - anchor_pose.point.y(),
                              mp->point.x() - anchor_pose.point.x());
    double cost =
        std::pow(angle, this->config_.angle_exponent_) * config_.angle_gain_ +
        std::pow(dist, this->config_.distance_exponent_) * config_.distance_gain_;
    pq.push({cost, mp});
  }

  // Get the closest midpoints in front of the car
  std::vector<std::shared_ptr<Midpoint>> candidate_points;  
  int count = 0;
  while (count < num_candidates && !pq.empty()) {
    candidate_points.push_back(pq.top().second);
    pq.pop();
    count++;
  }

  return candidate_points;
}

std::pair<std::shared_ptr<Midpoint>, std::shared_ptr<Midpoint>>  
PathCalculation::select_starting_midpoints() {
  std::pair<std::shared_ptr<Midpoint>, std::shared_ptr<Midpoint>> result{nullptr, nullptr};  

  Midpoint anchor_pose_midpoint{  
      Point(initial_pose_.position.x, initial_pose_.position.y), nullptr, nullptr};

  // Get candidate midpoints
  anchor_pose_midpoint.close_points = select_candidate_midpoints(anchor_pose_midpoint, 8);

  double best_cost = std::numeric_limits<double>::max();
  for (const auto& first : anchor_pose_midpoint.close_points) {
    auto anchor_mp = std::make_shared<Midpoint>(anchor_pose_midpoint);  
    auto [cost, second] = find_best_next_midpoint(this->config_.search_depth_, anchor_mp, first,
                                   std::numeric_limits<double>::max());
    cost += std::pow(std::sqrt(std::pow(first->point.x() - anchor_pose_midpoint.point.x(), 2) +
                               std::pow(first->point.y() - anchor_pose_midpoint.point.y(), 2)),
                     this->config_.distance_exponent_) *
            this->config_.distance_gain_;
    if (cost < best_cost) {
      result.first = first;
      result.second = second;
      best_cost = cost;
    }
  }

  return result;
}

std::shared_ptr<Midpoint> PathCalculation::find_nearest_midpoint(const Point& target) {  
  double min_dist_sq = config_.tolerance_ * config_.tolerance_;
  std::shared_ptr<Midpoint> nearest = nullptr;  

  for (const auto& [pt, mp] : point_to_midpoint_) {
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

int PathCalculation::find_best_loop_closure(const std::vector<PathPoint>& path) {
  if (path.size() < 3) {
    return path.size() - 1;
  }

  PathPoint first_point = path[0];
  double min_cost = std::numeric_limits<double>::max();
  int best_cutoff_index = path.size() - 1;  // Default to last point

  // Check each point in the path to find the best one to close the loop
  for (int i = 2; i < static_cast<int>(path.size()); ++i) {
    PathPoint current = path[i];
    PathPoint previous = path[i - 1];

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

  return best_cutoff_index;
}

std::vector<PathPoint> PathCalculation::add_interpolated_points(
    const PathPoint& start,
    const PathPoint& end,
    int num_points) {
  
  std::vector<PathPoint> interpolated;
  
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

std::vector<PathPoint> PathCalculation::calculate_trackdrive(std::vector<Cone>& cone_array) {
  vector<PathPoint> result = calculate_path(cone_array);

  // Check if we have enough points to form a loop
  if (result.size() < 3) {
    RCLCPP_WARN(rclcpp::get_logger("planning"), "Not enough points to create trackdrive loop");
    return result;
  }

  // Find the best point to close the loop
  int best_cutoff_index = find_best_loop_closure(result);

  // Trim the path to the best cutoff point
  result.erase(result.begin() + best_cutoff_index + 1, result.end());

  // Add interpolated points between the last point and the first point
  if (result.size() > 0) {
    PathPoint last_point = result.back();
    PathPoint first_point = result[0];
    
    std::vector<PathPoint> interpolated = add_interpolated_points(last_point, first_point, 4);
    result.insert(result.end(), interpolated.begin(), interpolated.end());
  }

  // Close the loop by adding the first point again
  result.push_back(result[0]);

  // Add overlap points (10 points or as many as available)
  int overlap_count = std::min(10, static_cast<int>(result.size()) - 1);
  for (int i = 1; i <= overlap_count; ++i) {
    result.push_back(result[i]);
  }

  return result;
}

std::vector<PathPoint> PathCalculation::get_path_to_car() const {
  std::vector<PathPoint> path_points;

  for (const auto& pt : path_to_car) {
    (void)path_points.emplace_back(pt.x(), pt.y());
  }

  return path_points;
}

const std::vector<std::pair<Point, Point>>& PathCalculation::get_triangulations() const {
  return midpoint_generator_.get_triangulations();
}