#include "statistics/collision_statistics.hpp"

CollisionStatistics::CollisionStatistics(const Track& track, const std::string& discipline,
                                         const std::string& car_parameters_config)
    : cones_(track.get_cones()) {
  load_cone_collision_config();
  load_penalty_config();
  penalty_time_per_cone_ = penalty_time_for_discipline(discipline);
  car_hitboxes_ = load_car_hitboxes(car_parameters_config);
}

void CollisionStatistics::load_cone_collision_config() {
  const std::string path =
      std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/cones/config.yaml";
  const YAML::Node config = YAML::LoadFile(path);
  const YAML::Node collision = config["collision"];
  if (!collision) {
    return;
  }

  standard_cone_radius_m_ = collision["standard_radius"].as<double>(standard_cone_radius_m_);
  large_cone_radius_m_ = collision["large_radius"].as<double>(large_cone_radius_m_);
}

void CollisionStatistics::load_penalty_config() {
  const std::string path =
      std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/ground/config.yaml";
  const YAML::Node config = YAML::LoadFile(path);
  const YAML::Node penalties = config["lap_finish"]["penalties"];
  if (!penalties) {
    return;
  }

  default_cone_penalty_time_s_ = penalties["default_cone"].as<double>(default_cone_penalty_time_s_);
  skidpad_cone_penalty_time_s_ = penalties["skidpad_cone"].as<double>(skidpad_cone_penalty_time_s_);
}

void CollisionStatistics::reset_lap(StatisticsSnapshot& snapshot) {
  snapshot.current_lap_cones_hit = 0;
  hit_cone_count_ = 0;
  cone_was_hit_.assign(cones_.size(), false);
}

void CollisionStatistics::update(const VehicleModelSnapshot& vehicle_snapshot,
                                 bool lap_timing_started, StatisticsSnapshot& snapshot) {
  if (!lap_timing_started) {
    return;
  }

  for (std::size_t i = 0; i < cones_.size(); ++i) {
    if (cone_was_hit_[i] || !collides_with_cone(vehicle_snapshot, cones_[i])) {
      continue;
    }

    cone_was_hit_[i] = true;
    ++hit_cone_count_;
    snapshot.current_lap_cones_hit = static_cast<int>(hit_cone_count_);
  }
}

double CollisionStatistics::current_lap_penalty_time() const {
  return static_cast<double>(hit_cone_count_) * penalty_time_per_cone_;
}

std::vector<common_lib::structures::Cone> CollisionStatistics::recently_hit_cones() const {
  std::vector<common_lib::structures::Cone> hit_cones;
  hit_cones.reserve(hit_cone_count_);

  for (std::size_t i = 0; i < cones_.size(); ++i) {
    if (cone_was_hit_[i]) {
      hit_cones.push_back(cones_[i]);
    }
  }

  return hit_cones;
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

  const std::string path = std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/car/" +
                           car_parameters_config + "/config.yaml";

  const YAML::Node config = YAML::LoadFile(path);
  const YAML::Node yaml_boxes = config["hitboxes"]["boxes"];

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
