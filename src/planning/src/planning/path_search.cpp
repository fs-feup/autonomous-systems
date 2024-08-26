#include "planning/path_search.hpp"

double PathSearch::calculate_cost(const PathPoint& next_cone, const PathPoint& last_cone,
                                  const TwoDVector& previous_to_last_vector,
                                  const double& colored_to_input_cones_ratio) const {
  AngleAndNorms angle_and_norms = common_lib::maths::angle_and_norms(
      previous_to_last_vector, Position{next_cone.position.x - last_cone.position.x,
                                        next_cone.position.y - last_cone.position.y});
  double distance = angle_and_norms.norm2_;
  double angle = angle_and_norms.angle_;
  double cost = this->config_.distance_weight_ * pow(distance, this->config_.distance_exponent_) +
                this->config_.angle_weight_ * pow(angle, this->config_.angle_exponent_) +
                this->config_.npoints_weight_ * colored_to_input_cones_ratio;
  return cost;
}

bool PathSearch::try_to_color_next_cone(
    std::unordered_set<PathPoint, std::hash<PathPoint>>& remaining_pathpoints,
    std::vector<PathPoint>& current_path, int& n_current_pathpoints,
    const int n_input_cones) const {
  double min_cost = std::numeric_limits<double>::max();
  PathPoint cheapest_point;
  const PathPoint last_cone = current_path.back();
  const PathPoint second_last_cone = current_path[current_path.size() - 2];
  const TwoDVector last_vector = {last_cone.position.x - second_last_cone.position.x,
                                  last_cone.position.y - second_last_cone.position.y};
  for (const auto& cone : remaining_pathpoints) {
    double cost = calculate_cost(
        cone, last_cone, last_vector,
        static_cast<double>(n_current_pathpoints) / static_cast<double>(n_input_cones));
    // TODO: put this value as a parameter
    if (cost < min_cost && cone.position.euclidean_distance(last_cone.position) < 7 &&
        cost < this->config_.max_cost_) {
      min_cost = cost;
      cheapest_point = cone;
    }
  }
  if (min_cost < std::numeric_limits<double>::max()) {
    remaining_pathpoints.erase(cheapest_point);
    current_path.push_back(cheapest_point);
    n_current_pathpoints++;
    return true;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("PathSearch"), "Could not find next path point %i",
                n_current_pathpoints);
    return false;
  }
}

PathPoint PathSearch::find_initial_cone(
    const std::unordered_set<PathPoint, std::hash<PathPoint>>& cones, const Pose& car_pose) const {
  Position expected_cone_position = car_pose.position;
  PathPoint initial_cone =
      *std::min_element(cones.begin(), cones.end(),
                        [&expected_cone_position](const PathPoint& cone1, const PathPoint& cone2) {
                          return cone1.position.euclidean_distance(expected_cone_position) <
                                 cone2.position.euclidean_distance(expected_cone_position);
                        });
  return initial_cone;
}

void PathSearch::place_second_cones(
    std::unordered_set<PathPoint, std::hash<PathPoint>>& input_pathpoints,
    std::vector<PathPoint>& final_path, const Pose& car_pose, int& n_current_pathpoints) const {
  PathPoint first_pathpoint = final_path.front();

  double car_orientation = car_pose.orientation;
  double min_cost = std::numeric_limits<double>::max();

  PathPoint second_pathpoint;
  for (const auto& cone : input_pathpoints) {
    double segment_orientation = atan2(cone.position.y - first_pathpoint.position.y,
                                       cone.position.x - first_pathpoint.position.x);
    segment_orientation = std::fmod(segment_orientation + 2.0 * M_PI, 2.0 * M_PI);
    double orientation_difference = std::fabs(car_orientation - segment_orientation);
    if (orientation_difference > 2.0 * M_PI / 5.0 && orientation_difference < 8.0 * M_PI / 5.0) {
      continue;
    }
    double cost = std::pow(cone.position.euclidean_distance(first_pathpoint.position),
                           config_.distance_exponent_) *
                      config_.distance_weight_ +
                  std::pow(orientation_difference, config_.angle_exponent_) * config_.angle_weight_;
    if (cost < min_cost) {
      min_cost = cost;
      second_pathpoint = cone;
    }
  }
  if (min_cost < std::numeric_limits<double>::max()) {
    final_path.push_back(second_pathpoint);
    input_pathpoints.erase(second_pathpoint);
    n_current_pathpoints++;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("PathSearch"), "Could not find second path point");
  }
}

void PathSearch::place_initial_cones(
    std::unordered_set<PathPoint, std::hash<PathPoint>>& input_pathpoints,
    std::vector<PathPoint>& final_path, const Pose& car_pose, int& n_current_pathpoints) const {
  PathPoint initial_pathpoint = find_initial_cone(input_pathpoints, car_pose);
  input_pathpoints.erase(initial_pathpoint);
  final_path.push_back(initial_pathpoint);
  n_current_pathpoints++;

  place_second_cones(input_pathpoints, final_path, car_pose, n_current_pathpoints);
}

std::vector<PathPoint> PathSearch::find_path(const std::vector<PathPoint>& path_points,
                                             const Pose& car_pose) const {
  if (config_.using_cone_colouring_) {
    return path_points;
  }
  if (path_points.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("Planning : PathSearch"),
                "Too few points to perform search: %ld", path_points.size());
    return {};
  }
  std::vector<PathPoint> final_path;
  final_path.reserve(path_points.size());
  int n_current_pathpoints = 0;
  const auto n_input_points = static_cast<int>(path_points.size());
  std::unordered_set<PathPoint, std::hash<PathPoint>> remaining_pathpoints(path_points.begin(),
                                                                           path_points.end());
  place_initial_cones(remaining_pathpoints, final_path, car_pose, n_current_pathpoints);
  // Color blue cones
  while (try_to_color_next_cone(remaining_pathpoints, final_path, n_current_pathpoints,
                                n_input_points)) {
    // keep coloring blue cones while the function "try_to_color_next_cone" returns true (i.e. a
    // suitble cone is found)
  }
  if (final_path.size() < 5) {
    RCLCPP_DEBUG(rclcpp::get_logger("Planning : PathSearch"), "Not enough path points found: %ld",
                 final_path.size());
  }
  return final_path;
}
