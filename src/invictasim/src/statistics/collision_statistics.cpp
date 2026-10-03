#include "statistics/collision_statistics.hpp"

#include <algorithm>
#include <cmath>

#include "config/resource_paths.hpp"

CollisionStatistics::CollisionStatistics(const Track& track, const std::string& discipline,
                                         const std::string& car_parameters_config)
    : cones_(track.get_cones()) {
  load_cone_collision_config();
  load_penalty_config();
  penalty_time_per_cone_ = penalty_time_for_discipline(discipline);
  car_hitboxes_ = load_car_hitboxes(car_parameters_config);
  compute_collision_search_radius();
}

void CollisionStatistics::load_cone_collision_config() {
  const YAML::Node collision = invictasim::resources::load("meshes/cones/config.yaml")["collision"];
  if (!collision) {
    return;
  }

  standard_cone_radius_m_ = collision["standard_radius"].as<double>(standard_cone_radius_m_);
  large_cone_radius_m_ = collision["large_radius"].as<double>(large_cone_radius_m_);
}

void CollisionStatistics::load_penalty_config() {
  const YAML::Node penalties =
      invictasim::resources::load("meshes/ground/config.yaml")["lap_finish"]["penalties"];
  if (!penalties) {
    return;
  }

  default_cone_penalty_time_s_ = penalties["default_cone"].as<double>(default_cone_penalty_time_s_);
  skidpad_cone_penalty_time_s_ = penalties["skidpad_cone"].as<double>(skidpad_cone_penalty_time_s_);
}

void CollisionStatistics::reset_lap(StatisticsSnapshot& snapshot) {
  snapshot.current_lap_cones_hit = 0;
  cones_hit_this_lap_.clear();
  cone_was_hit_.assign(cones_.size(), false);
}

void CollisionStatistics::update(const VehicleModelSnapshot& vehicle_snapshot,
                                 bool lap_timing_started, StatisticsSnapshot& snapshot) {
  if (!lap_timing_started) {
    return;
  }

  // Reject by distance first. This runs every simulation step against the whole cone list, and on
  // an endurance layout only a handful of cones are ever within reach of a hitbox.
  const double search_radius_squared = collision_search_radius_m_ * collision_search_radius_m_;

  for (std::size_t i = 0; i < cones_.size(); ++i) {
    if (cone_was_hit_[i]) {
      continue;
    }

    const double dx = cones_[i].position.x - vehicle_snapshot.x;
    const double dy = cones_[i].position.y - vehicle_snapshot.y;
    if (dx * dx + dy * dy > search_radius_squared) {
      continue;
    }

    if (!collides_with_cone(vehicle_snapshot, cones_[i])) {
      continue;
    }

    cone_was_hit_[i] = true;
    cones_hit_this_lap_.push_back(cones_[i]);
    snapshot.current_lap_cones_hit = static_cast<int>(cones_hit_this_lap_.size());
  }
}

void CollisionStatistics::compute_collision_search_radius() {
  // Distance from the car origin to the furthest hitbox corner, plus the largest cone radius.
  double furthest_corner = 0.0;
  for (const Hitbox& hitbox : car_hitboxes_) {
    furthest_corner =
        std::max(furthest_corner, std::hypot(std::abs(hitbox.center_x) + hitbox.half_length,
                                             std::abs(hitbox.center_y) + hitbox.half_width));
  }
  collision_search_radius_m_ =
      furthest_corner + std::max(standard_cone_radius_m_, large_cone_radius_m_);
}

double CollisionStatistics::current_lap_penalty_time() const {
  return static_cast<double>(cones_hit_this_lap_.size()) * penalty_time_per_cone_;
}

const std::vector<common_lib::structures::Cone>& CollisionStatistics::cones_hit_this_lap() const {
  return cones_hit_this_lap_;
}

double CollisionStatistics::penalty_time_for_discipline(std::string discipline) const {
  std::transform(discipline.begin(), discipline.end(), discipline.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return discipline == "skidpad" ? skidpad_cone_penalty_time_s_ : default_cone_penalty_time_s_;
}

std::vector<Hitbox> CollisionStatistics::load_car_hitboxes(
    const std::string& car_parameters_config) const {
  std::vector<Hitbox> hitboxes;
  if (car_parameters_config.empty()) {
    return hitboxes;
  }

  const YAML::Node yaml_boxes = invictasim::resources::load("meshes/car/" + car_parameters_config +
                                                            "/config.yaml")["hitboxes"]["boxes"];
  if (!yaml_boxes || !yaml_boxes.IsSequence()) {
    return hitboxes;
  }

  for (const auto& node : yaml_boxes) {
    const double length = node["length"].as<double>();
    const double width = node["width"].as<double>();

    hitboxes.push_back({node["center_x"].as<double>(0.0), node["center_y"].as<double>(0.0),
                        length * 0.5, width * 0.5});
  }

  return hitboxes;
}

double CollisionStatistics::cone_radius(const common_lib::structures::Cone& cone) const {
  return cone.is_large || cone.color == common_lib::competition_logic::Color::LARGE_ORANGE
             ? large_cone_radius_m_
             : standard_cone_radius_m_;
}

bool CollisionStatistics::collides_with_cone(const VehicleModelSnapshot& vehicle_snapshot,
                                             const common_lib::structures::Cone& cone) const {
  const double dx = cone.position.x - vehicle_snapshot.x;
  const double dy = cone.position.y - vehicle_snapshot.y;
  const double cos_yaw = std::cos(vehicle_snapshot.yaw);
  const double sin_yaw = std::sin(vehicle_snapshot.yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  const double radius = cone_radius(cone);
  for (const Hitbox& hitbox : car_hitboxes_) {
    const double min_x = hitbox.center_x - hitbox.half_length;
    const double max_x = hitbox.center_x + hitbox.half_length;
    const double min_y = hitbox.center_y - hitbox.half_width;
    const double max_y = hitbox.center_y + hitbox.half_width;
    const double closest_x = std::clamp(local_x, min_x, max_x);
    const double closest_y = std::clamp(local_y, min_y, max_y);
    if (std::hypot(local_x - closest_x, local_y - closest_y) <= radius) {
      return true;
    }
  }

  return false;
}
