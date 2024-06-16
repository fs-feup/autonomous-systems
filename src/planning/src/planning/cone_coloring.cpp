#include "planning/cone_coloring.hpp"

void ConeColoring::remove_duplicates(std::vector<Cone>& cones) {
  std::unordered_set<Cone, std::hash<Cone>, ConeAproxEqual> unique_cones;
  std::vector<Cone> result;
  for (const auto& cone : cones) {
    if (unique_cones.insert(cone).second) {
      result.push_back(cone);
    }
  }

  cones = result;
}

Position ConeColoring::expected_initial_cone_position(const Pose& car_pose,
                                                      const TrackSide& track_side) {
  double distance_to_car = 2.0;
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

Cone ConeColoring::find_initial_cone(
    const std::unordered_set<Cone, std::hash<Cone>, ConeAproxEqual>& cones, const Pose& car_pose,
    const TrackSide track_side) {
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

Cone ConeColoring::virtual_cone_from_initial_cone(const Cone& initial_cone, const Pose& car_pose) {
  double distance_to_virtual_cone = 2.0;
  return Cone{
      Position{initial_cone.position.x - distance_to_virtual_cone * cos(car_pose.orientation),
               initial_cone.position.y - distance_to_virtual_cone * sin(car_pose.orientation)},
      initial_cone.color, 1.0};
}

void ConeColoring::place_initial_cones(
    std::unordered_set<Cone, std::hash<Cone>, ConeAproxEqual>& uncolored_cones,
    std::vector<Cone>& colored_blue_cones, std::vector<Cone>& colored_yellow_cones,
    const Pose& car_pose, int& n_colored_cones) {
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
                                    const double& colored_to_input_cones_ratio) {
  AngleAndNorms angle_and_norms = common_lib::maths::angle_and_norms(
      previous_to_last_vector, Position{next_cone.position.x - last_cone.position.x,
                                        next_cone.position.y - last_cone.position.y});
  double distance = angle_and_norms.norm2;
  double angle = angle_and_norms.angle;
  double cost = this->config_.distance_weight * pow(distance, this->config_.distance_exponent) +
                this->config_.angle_weight * pow(angle, this->config_.angle_exponent) +
                this->config_.ncones_weight * colored_to_input_cones_ratio;
  return cost;
}

bool ConeColoring::try_to_color_next_cone(
    std::unordered_set<Cone, std::hash<Cone>, ConeAproxEqual>& uncolored_cones,
    std::vector<Cone>& colored_cones, int& n_colored_cones, const int n_input_cones) {
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
        cost < this->config_.max_cost) {
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
  if (cones.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("Planning : ConeColoring"), "No cones recieved to be colored");
    return {};
  }
  std::vector<Cone> colored_blue_cones;
  std::vector<Cone> colored_yellow_cones;
  int n_colored_cones = 0;
  const auto n_input_cones = static_cast<int>(cones.size());
  std::unordered_set<Cone, std::hash<Cone>, ConeAproxEqual> uncolored_cones(cones.begin(),
                                                                            cones.end());
  place_initial_cones(uncolored_cones, colored_blue_cones, colored_yellow_cones, car_pose,
                      n_colored_cones);
  // Color blue cones
  while (
      try_to_color_next_cone(uncolored_cones, colored_blue_cones, n_colored_cones, n_input_cones)) {
  }
  // Color yellow cones
  while (try_to_color_next_cone(uncolored_cones, colored_yellow_cones, n_colored_cones,
                                n_input_cones)) {
  }
  return {colored_blue_cones, colored_yellow_cones};
}
