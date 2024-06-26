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
  if (track_side == TrackSide::LEFT) {
    initial_cone.color = Color::BLUE;
  } else if (track_side == TrackSide::RIGHT) {
    initial_cone.color = Color::YELLOW;
  }
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
                                       std::vector<Cone>& colored_blue_cones,
                                       std::vector<Cone>& colored_yellow_cones,
                                       const Pose& car_pose, int& n_colored_cones) const {
  Cone initial_cone_left = find_initial_cone(uncolored_cones, car_pose, TrackSide::LEFT);
  uncolored_cones.erase(initial_cone_left);
  colored_blue_cones.push_back(initial_cone_left);
  n_colored_cones++;

  Cone initial_cone_right = find_initial_cone(uncolored_cones, car_pose, TrackSide::RIGHT);
  uncolored_cones.erase(initial_cone_right);
  colored_yellow_cones.push_back(initial_cone_right);
  n_colored_cones++;

  Cone virtual_cone_left = virtual_cone_from_initial_cone(initial_cone_left, car_pose);
  Cone virtual_cone_right = virtual_cone_from_initial_cone(initial_cone_right, car_pose);
  colored_blue_cones.insert(colored_blue_cones.begin(), virtual_cone_left);
  colored_yellow_cones.insert(colored_yellow_cones.begin(), virtual_cone_right);
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
    int& n_colored_cones, const int n_input_cones) const {
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
    if (cost < min_cost && cone.position.euclidean_distance(last_cone.position) < 5 &&
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
  if (cones.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("Planning : ConeColoring"),
                "Not enough cones recieved to be colored: %ld", cones.size());
    return {};
  }
  std::vector<Cone> colored_blue_cones;
  std::vector<Cone> colored_yellow_cones;
  colored_blue_cones.reserve(cones.size() / 2);
  colored_yellow_cones.reserve(cones.size() / 2);
  int n_colored_cones = 0;
  const auto n_input_cones = static_cast<int>(cones.size());
  std::unordered_set<Cone, std::hash<Cone>> uncolored_cones(cones.begin(), cones.end());
  place_initial_cones(uncolored_cones, colored_blue_cones, colored_yellow_cones, car_pose,
                      n_colored_cones);
  // Color blue cones
  while (
      try_to_color_next_cone(uncolored_cones, colored_blue_cones, n_colored_cones, n_input_cones)) {
    // keep coloring yellow cones while the function "try_to_color_next_cone" returns true (i.e. a
    // suitble cone is found)
  }
  // Color yellow cones
  while (try_to_color_next_cone(uncolored_cones, colored_yellow_cones, n_colored_cones,
                                n_input_cones)) {
    // keep coloring yellow cones while the function "try_to_color_next_cone" returns true (i.e. a
    // suitble cone is found)
  }
  colored_blue_cones.erase(colored_blue_cones.begin());
  colored_yellow_cones.erase(colored_yellow_cones.begin());
  return {colored_blue_cones, colored_yellow_cones};
}
