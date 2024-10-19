#include "planning/cone_coloring.hpp"

void ConeColoring::remove_duplicates(std::vector<Cone>& cones) const {
  std::unordered_set<Cone, std::hash<Cone>> unique_cones;
  std::vector<Cone> result;
  result.reserve(cones.size());
  for (const auto& cone : cones) {
    if (unique_cones.insert(cone).second) {
      result.push_back(cone);
    }
  }

  cones = result;
}

std::vector<Cone> ConeColoring::filter_previously_colored_cones(const std::vector<Cone>& cones) {
  std::vector<Cone> uncolored_cones;
  bool seen;
  for (const auto& cone : cones) {
    seen = false;
    if (!seen) {
      for (auto& colored_cone : this->colored_blue_cones_) {
        if (cone.position.euclidean_distance(colored_cone.position) < 0.6) {
          seen = true;
          colored_cone.position.x = cone.position.x;
          colored_cone.position.y = cone.position.y;
          break;
        }
      }
    }

    if (!seen) {
      for (auto& colored_cone : this->colored_yellow_cones_) {
        if (cone.position.euclidean_distance(colored_cone.position) < 0.6) {
          seen = true;
          colored_cone.position.x = cone.position.x;
          colored_cone.position.y = cone.position.y;
          break;
        }
      }
    }
    if (!seen) {
      uncolored_cones.push_back(cone);
    }
  }
  return uncolored_cones;
}

Position ConeColoring::expected_initial_cone_position(const Pose& car_pose,
                                                      const TrackSide& track_side) const {
  constexpr double distance_to_car = 2.0;
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

Cone ConeColoring::find_initial_cone(const std::unordered_set<Cone, std::hash<Cone>>& cones,
                                     const Pose& car_pose, const TrackSide track_side) const {
  Position expected_cone_position = expected_initial_cone_position(car_pose, track_side);
  Cone initial_cone = *std::min_element(
      cones.begin(), cones.end(), [&expected_cone_position](const Cone& cone1, const Cone& cone2) {
        return cone1.position.euclidean_distance(expected_cone_position) <
               cone2.position.euclidean_distance(expected_cone_position);
      });
  return initial_cone;
}

Cone ConeColoring::virtual_cone_from_initial_cone(const Cone& initial_cone,
                                                  const Pose& car_pose) const {
  constexpr double distance_to_virtual_cone = 2.0;
  return Cone{
      Position{initial_cone.position.x - distance_to_virtual_cone * cos(car_pose.orientation),
               initial_cone.position.y - distance_to_virtual_cone * sin(car_pose.orientation)},
      initial_cone.color, 1.0};
}

void ConeColoring::place_initial_cones(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones,
                                       const Pose& car_pose, int& n_colored_cones) {
  Cone initial_cone_left = find_initial_cone(uncolored_cones, car_pose, TrackSide::LEFT);
  uncolored_cones.erase(initial_cone_left);
  colored_blue_cones_.push_back(initial_cone_left);
  n_colored_cones++;

  Cone initial_cone_right = find_initial_cone(uncolored_cones, car_pose, TrackSide::RIGHT);
  uncolored_cones.erase(initial_cone_right);
  colored_yellow_cones_.push_back(initial_cone_right);
  n_colored_cones++;

  place_second_cones(uncolored_cones, car_pose, n_colored_cones);
}

void ConeColoring::place_second_cones(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones,
                                      const Pose& car_pose, int& n_colored_cones) {
  Cone first_left_cone = colored_blue_cones_.front();
  Cone first_right_cone = colored_yellow_cones_.front();

  double car_orientation = car_pose.orientation;
  double min_distance = std::numeric_limits<double>::max();

  Cone second_left_cone;
  for (const auto& cone : uncolored_cones) {
    double segment_orientation = atan2(cone.position.y - first_left_cone.position.y,
                                       cone.position.x - first_left_cone.position.x);
    segment_orientation = std::fmod(segment_orientation + 2.0 * M_PI, 2.0 * M_PI);
    double orientation_difference = std::fabs(car_orientation - segment_orientation);
    if (orientation_difference > 2.0 * M_PI / 5.0 && orientation_difference < 8.0 * M_PI / 5.0) {
      continue;
    }
    double distance = cone.position.euclidean_distance(first_left_cone.position);
    if (distance < min_distance) {
      min_distance = distance;
      second_left_cone = cone;
    }
  }
  if (min_distance < std::numeric_limits<double>::max()) {
    colored_blue_cones_.push_back(second_left_cone);
    uncolored_cones.erase(second_left_cone);
    n_colored_cones++;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ConeColoring"), "Could not find second left cone");
  }

  min_distance = std::numeric_limits<double>::max();
  Cone second_right_cone;
  for (const auto& cone : uncolored_cones) {
    double segment_orientation = atan2(cone.position.y - first_right_cone.position.y,
                                       cone.position.x - first_right_cone.position.x);
    segment_orientation = std::fmod(segment_orientation + 2.0 * M_PI, 2.0 * M_PI);
    double orientation_difference = std::fabs(car_orientation - segment_orientation);
    if (orientation_difference > 2.0 * M_PI / 5.0 && orientation_difference < 8.0 * M_PI / 5.0) {
      continue;
    }
    double distance = cone.position.euclidean_distance(first_right_cone.position);
    if (distance < min_distance) {
      min_distance = distance;
      second_right_cone = cone;
    }
  }
  if (min_distance < std::numeric_limits<double>::max()) {
    colored_yellow_cones_.push_back(second_right_cone);
    uncolored_cones.erase(second_right_cone);
    n_colored_cones++;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ConeColoring"), "Could not find second right cone");
  }
}

double ConeColoring::calculate_cost(const Cone& next_cone, const Cone& last_cone,
                                    const TwoDVector& previous_to_last_vector,
                                    const double& colored_to_input_cones_ratio) const {
  AngleAndNorms angle_and_norms = common_lib::maths::angle_and_norms(
      previous_to_last_vector, Position{next_cone.position.x - last_cone.position.x,
                                        next_cone.position.y - last_cone.position.y});
  double distance = angle_and_norms.norm2_;
  double angle = angle_and_norms.angle_;
  double cost = this->config_.distance_weight_ * pow(distance, this->config_.distance_exponent_) +
                this->config_.angle_weight_ * pow(angle, this->config_.angle_exponent_) +
                this->config_.ncones_weight_ * colored_to_input_cones_ratio;
  return cost;
}

bool ConeColoring::try_to_color_next_cone(
    std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones, std::vector<Cone>& colored_cones,
    int& n_colored_cones, const int n_input_cones) {
  double min_cost = std::numeric_limits<double>::max();
  Cone cheapest_cone;
  const Cone last_cone = colored_cones.back();
  const Cone second_last_cone = colored_cones[colored_cones.size() - 2];
  const TwoDVector last_vector = {last_cone.position.x - second_last_cone.position.x,
                                  last_cone.position.y - second_last_cone.position.y};
  for (const auto& cone : uncolored_cones) {
    double cost =
        calculate_cost(cone, last_cone, last_vector,
                       static_cast<double>(n_colored_cones) / static_cast<double>(n_input_cones));
    // TODO: put this value as a parameter
    if (cost < min_cost && cone.position.euclidean_distance(last_cone.position) < 7 &&
        cone.position.euclidean_distance(last_cone.position) > 0.8 &&
        cost < this->config_.max_cost_) {
      min_cost = cost;
      cheapest_cone = cone;
    }
  }
  if (min_cost < std::numeric_limits<double>::max()) {
    uncolored_cones.erase(cheapest_cone);
    colored_cones.push_back(cheapest_cone);
    n_colored_cones++;
    return true;
  } else {
    return false;
  }
}

std::pair<std::vector<Cone>, std::vector<Cone>> ConeColoring::color_cones(std::vector<Cone> cones,
                                                                          const Pose& car_pose) {
  remove_duplicates(cones);
  cones = filter_previously_colored_cones(cones);

  int n_colored_cones = 0;
  const auto n_input_cones = static_cast<int>(cones.size());
  RCLCPP_DEBUG(rclcpp::get_logger("Planning : ConeColoring"), "Number of received cones: %d",
               n_input_cones);
  std::unordered_set<Cone, std::hash<Cone>> uncolored_cones(cones.begin(), cones.end());

  if (this->colored_blue_cones_.empty() || this->colored_yellow_cones_.empty()) {
    if (cones.size() < 4) {
      RCLCPP_WARN(rclcpp::get_logger("Planning: Cone Coloring"), "No cones found yet.");
      return {{}, {}};
    } else {
      place_initial_cones(uncolored_cones, car_pose, n_colored_cones);
    }
  }

  bool colouring_blue_cones = true, colouring_yellow_cones = true;
  // Color blue cones
  while (colouring_blue_cones || colouring_yellow_cones) {
    // keep coloring yellow cones while the function "try_to_color_next_cone" returns true (i.e. a
    // suitble cone is found)
    colouring_blue_cones = try_to_color_next_cone(uncolored_cones, this->colored_blue_cones_,
                                                  n_colored_cones, n_input_cones);
    colouring_yellow_cones = try_to_color_next_cone(uncolored_cones, this->colored_yellow_cones_,
                                                    n_colored_cones, n_input_cones);
  }
  //// Color yellow cones
  // while (try_to_color_next_cone(uncolored_cones, colored_yellow_cones, n_colored_cones,
  //                               n_input_cones)) {
  //   // keep coloring yellow cones while the function "try_to_color_next_cone" returns true (i.e.
  //   a
  //   // suitble cone is found)
  // }
  // colored_blue_cones.erase(colored_blue_cones.begin());
  // colored_yellow_cones.erase(colored_yellow_cones.begin());
  if (colored_blue_cones_.size() < 5) {
    RCLCPP_DEBUG(rclcpp::get_logger("Planning : ConeColoring"), "Not enough blue cones found: %ld",
                 colored_blue_cones_.size());
  }
  if (colored_yellow_cones_.size() < 5) {
    RCLCPP_DEBUG(rclcpp::get_logger("Planning : ConeColoring"),
                 "Not enough yellow cones found: %ld", colored_yellow_cones_.size());
  }
  return {this->colored_blue_cones_, this->colored_yellow_cones_};
}
