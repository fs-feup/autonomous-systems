#include "planning/path_finder.hpp"

PathFinder::PathFinder(ConeColoringConfig& config) : config_(config) {}

Position PathFinder::expected_initial_cone_position(const Pose& car_pose,
                                                    const TrackSide& track_side) const {
  constexpr double distance_to_car = 1.5;
  if (track_side == TrackSide::LEFT) {
    return Position{car_pose.position.x - distance_to_car * sin(car_pose.orientation),
                    car_pose.position.y + distance_to_car * cos(car_pose.orientation)};
  } else if (track_side == TrackSide::RIGHT) {
    return Position{car_pose.position.x + distance_to_car * sin(car_pose.orientation),
                    car_pose.position.y - distance_to_car * cos(car_pose.orientation)};
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ConeColoring"), "Invalid track side");
    return Position{0, 0};
  }
}

void PathFinder::find_first_point(std::vector<PathPoint*>& mid_points, const Pose& car_pose) {
  if (mid_points.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("PathFinder"), "No mid points available");
    return;
  }

  PathPoint* closest_point = mid_points[0];
  double min_distance = std::numeric_limits<double>::max();

  for (auto& point : mid_points) {
    double distance = std::hypot(point->position.x - car_pose.position.x,
                                 point->position.y - car_pose.position.y);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = point;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("planning"), "first point: (%f, %f)\n", closest_point->position.x,
              closest_point->position.y);

  path_.push_back(closest_point);
}

double PathFinder::calculate_cost(const PathPoint* next_cone, const PathPoint* last_cone,
                                  const TwoDVector& previous_to_last_vector,
                                  const double& colored_to_input_cones_ratio) const {
  AngleAndNorms angle_and_norms = common_lib::maths::angle_and_norms(
      previous_to_last_vector, Position{next_cone->position.x - last_cone->position.x,
                                        next_cone->position.y - last_cone->position.y});
  double distance = angle_and_norms.norm2_;
  double angle = angle_and_norms.angle_;
  double cost = this->config_.distance_weight_ * pow(distance, this->config_.distance_exponent_) +
                this->config_.angle_weight_ * pow(angle, this->config_.angle_exponent_) +
                this->config_.ncones_weight_ * colored_to_input_cones_ratio;
  return cost;
}

double PathFinder::calculate_cost2(const PathPoint* next_cone, const PathPoint* last_cone,
                                   const double& colored_to_input_cones_ratio,
                                   const Pose& car_pose) const {
  double distance = next_cone->position.euclidean_distance(last_cone->position);
  double angle = std::atan2(next_cone->position.y - last_cone->position.y,
                            next_cone->position.x - last_cone->position.x) -
                 car_pose.orientation;
  double cost = this->config_.distance_weight_ * pow(distance, this->config_.distance_exponent_) +
                this->config_.angle_weight_ * pow(angle, this->config_.angle_exponent_) +
                this->config_.ncones_weight_ * colored_to_input_cones_ratio;
  return cost;
}

void PathFinder::find_second_point(std::vector<PathPoint*>& candidates, const Pose& car_pose,
                                   int& n_colored_cones, const int n_input_cones) {
  if (candidates.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("PathFinder"), "No candidates available");
    return;
  }
  PathPoint* second_cone;
  PathPoint* last_cone = path_.back();
  double min_cost = std::numeric_limits<double>::max();
  for (auto candidate : candidates) {
    double cost = calculate_cost2(
        candidate, last_cone,
        static_cast<double>(n_colored_cones) / static_cast<double>(n_input_cones), car_pose);
    if (cost < min_cost) {
      min_cost = cost;
      second_cone = candidate;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("planning"),
              "second point: (%f, %f), cone1 : (%f, %f), cone2 : (%f, %f)\n",
              second_cone->position.x, second_cone->position.y, second_cone->cone1->position.x,
              second_cone->cone1->position.y, second_cone->cone2->position.x,
              second_cone->cone2->position.y);
  path_.push_back(second_cone);
  n_colored_cones++;
}

bool PathFinder::try_to_color_next_cone(std::vector<PathPoint*>& uncolored_cones,
                                        std::vector<PathPoint*>& colored_cones,
                                        int& n_colored_cones, const int n_input_cones) {
  double min_cost = std::numeric_limits<double>::max();
  PathPoint* cheapest_cone;
  const PathPoint* last_cone = colored_cones.back();
  const PathPoint* second_last_cone = colored_cones[colored_cones.size() - 2];
  const TwoDVector last_vector = {last_cone->position.x - second_last_cone->position.x,
                                  last_cone->position.y - second_last_cone->position.y};
  for (const auto& cone : uncolored_cones) {
    double cost =
        calculate_cost(cone, last_cone, last_vector,
                       static_cast<double>(n_colored_cones) / static_cast<double>(n_input_cones));
    // TODO: put this value as a parameter
    if (cost < min_cost && cone->position.euclidean_distance(last_cone->position) < 7 &&
        cost < this->config_.max_cost_) {
      min_cost = cost;
      cheapest_cone = cone;
    }
  }
  if (min_cost < std::numeric_limits<double>::max()) {
    RCLCPP_INFO(rclcpp::get_logger("planning"),
                "selected point: (%f, %f) : cost: %f : cone1 : (%f, %f), cone2: (%f, %f)\n",
                cheapest_cone->position.x, cheapest_cone->position.y, min_cost,
                cheapest_cone->cone1->position.x, cheapest_cone->cone1->position.y,
                cheapest_cone->cone2->position.x, cheapest_cone->cone2->position.y);
    colored_cones.push_back(cheapest_cone);
    n_colored_cones++;
    return true;
  } else {
    return false;
  }
}

bool PathFinder::contains(PathPoint* point) {
  for (auto p : path_) {
    if (p->position.euclidean_distance(point->position) < 0.1) return true;
  }
  return false;
}

PathPoint* PathFinder::find_mid_point(std::vector<PathPoint*> mid_points, Cone* cone1,
                                      Cone* cone2) {
  for (auto m : mid_points) {
    if ((m->cone1 == cone1 && m->cone2 == cone2) || (m->cone2 == cone1 && m->cone1 == cone2))
      return m;
  }
  return nullptr;
}

std::vector<PathPoint*> PathFinder::find_candidates(std::vector<PathPoint*>& mid_points) {
  PathPoint* last_point = path_.back();
  Cone* cone1 = last_point->cone1;
  Cone* cone2 = last_point->cone2;
  std::vector<PathPoint*> candidates;

  std::vector<Cone*> neighbors1 = cone1->neighbors;
  std::vector<Cone*> neighbors2 = cone2->neighbors;

  std::sort(neighbors1.begin(), neighbors1.end());
  std::sort(neighbors2.begin(), neighbors2.end());

  std::vector<Cone*> common_neighbours;
  std::set_intersection(neighbors1.begin(), neighbors1.end(), neighbors2.begin(), neighbors2.end(),
                        std::back_inserter(common_neighbours));

  for (Cone* neighbor : common_neighbours) {
    PathPoint* p = find_mid_point(mid_points, cone1, neighbor);
    if (p != nullptr && !contains(p)) candidates.push_back(p);
    p = find_mid_point(mid_points, cone2, neighbor);
    if (p != nullptr && !contains(p)) candidates.push_back(p);
  }

  return candidates;
}

std::vector<PathPoint> PathFinder::find_path(std::vector<PathPoint*> cones, const Pose& car_pose) {
  RCLCPP_INFO(rclcpp::get_logger("path_finder"), "Car Pose: (%f, %f)", car_pose.position.x,
              car_pose.position.y);
  std::string track = "";
  for (auto p : cones) {
    if (p->position.euclidean_distance(car_pose.position) > 20) continue;
    track += "(" + std::to_string(p->position.x) + ", " + std::to_string(p->position.y) + "),";
  }
  track += "\n";
  RCLCPP_INFO(rclcpp::get_logger("Path_finder"), "Mid Points : %s", track.c_str());
  find_first_point(cones, car_pose);
  int n_input_cones = cones.size();
  int n_colored_cones = 1;
  find_second_point(cones, car_pose, n_colored_cones, n_input_cones);
  std::vector<PathPoint*> candidates = find_candidates(cones);
  std::string candidate_points = "";
  while (try_to_color_next_cone(candidates, path_, n_colored_cones, n_input_cones)) {
    for (auto p : candidates) {
      candidate_points +=
          "(" + std::to_string(p->position.x) + ", " + std::to_string(p->position.y) + "),";
    }
    candidates = find_candidates(cones);
  }
  RCLCPP_INFO(rclcpp::get_logger("Path_finder"), "Candidate Points : %s", candidate_points.c_str());
  std::vector<PathPoint> path;
  for (auto p : path_) {
    PathPoint path_point(p->position.x, p->position.y);
    path.push_back(path_point);
  }
  return path;
}