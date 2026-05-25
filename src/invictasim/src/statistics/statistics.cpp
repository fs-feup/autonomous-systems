#include "statistics/statistics.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

Statistics::Statistics(
    const Track& track,
    std::shared_ptr<const common_lib::car_parameters::CarParameters> car_parameters,
    const std::string& discipline, const std::string& car_parameters_config)
    : start_line_a_(track.get_start_line().first),
      start_line_b_(track.get_start_line().second),
      cones_(track.get_cones()) {
  if (car_parameters == nullptr) {
    throw std::invalid_argument("Statistics requires vehicle model car parameters");
  }

  load_cone_collision_config();
  load_ground_lap_finish_config();
  penalty_time_per_cone_ = penalty_time_for_discipline(discipline);
  car_hitboxes_ = load_car_hitboxes(car_parameters_config);
  reset();
}

std::vector<common_lib::structures::Cone> Statistics::get_recently_hit_cones() const {
  std::vector<common_lib::structures::Cone> hit_cones;
  hit_cones.reserve(hit_cone_indices_.size());

  for (const std::size_t cone_index : hit_cone_indices_) {
    if (cone_index < cones_.size()) {
      hit_cones.push_back(cones_[cone_index]);
    }
  }

  return hit_cones;
}

void Statistics::load_cone_collision_config() {
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

void Statistics::load_ground_lap_finish_config() {
  const std::string path =
      std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/ground/config.yaml";
  const YAML::Node config = YAML::LoadFile(path);
  const YAML::Node lap_finish = config["lap_finish"];
  if (!lap_finish) {
    return;
  }

  start_line_gate_margin_m_ =
      lap_finish["start_line_gate_margin"].as<double>(start_line_gate_margin_m_);
  minimum_lap_time_s_ = lap_finish["minimum_lap_time"].as<double>(minimum_lap_time_s_);
  minimum_lap_distance_m_ = lap_finish["minimum_lap_distance"].as<double>(minimum_lap_distance_m_);

  const YAML::Node penalties = lap_finish["penalties"];
  if (penalties) {
    default_cone_penalty_time_s_ =
        penalties["default_cone"].as<double>(default_cone_penalty_time_s_);
    skidpad_cone_penalty_time_s_ =
        penalties["skidpad_cone"].as<double>(skidpad_cone_penalty_time_s_);
  }
}

double Statistics::distance_between(const common_lib::structures::Position& a,
                                    const common_lib::structures::Position& b) const {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

std::pair<double, double> Statistics::tracking_error_and_objective_velocity(
    const common_lib::structures::Position& car_position,
    const std::vector<common_lib::structures::PathPoint>& path_points) const {
  if (path_points.empty()) {
    return {0.0, 0.0};
  }

  if (path_points.size() == 1) {
    return {distance_between(car_position, path_points.front().position),
            path_points.front().ideal_velocity};
  }

  double best_distance_sq = std::numeric_limits<double>::max();
  double best_velocity = 0.0;

  for (std::size_t i = 0; i + 1 < path_points.size(); ++i) {
    const auto& start = path_points[i];
    const auto& end = path_points[i + 1];
    const double segment_x = end.position.x - start.position.x;
    const double segment_y = end.position.y - start.position.y;
    const double segment_length_sq = segment_x * segment_x + segment_y * segment_y;

    double interpolation = 0.0;
    if (segment_length_sq > std::numeric_limits<double>::epsilon()) {
      const double car_x = car_position.x - start.position.x;
      const double car_y = car_position.y - start.position.y;
      interpolation =
          std::clamp((car_x * segment_x + car_y * segment_y) / segment_length_sq, 0.0, 1.0);
    }

    const double projected_x = start.position.x + interpolation * segment_x;
    const double projected_y = start.position.y + interpolation * segment_y;
    const double error_x = car_position.x - projected_x;
    const double error_y = car_position.y - projected_y;
    const double distance_sq = error_x * error_x + error_y * error_y;

    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_velocity =
          start.ideal_velocity + interpolation * (end.ideal_velocity - start.ideal_velocity);
    }
  }

  return {std::sqrt(best_distance_sq), best_velocity};
}

double Statistics::speed_from_snapshot(const VehicleModelSnapshot& snapshot) const {
  return std::hypot(snapshot.velocity_x, snapshot.velocity_y);
}

double Statistics::penalty_time_for_discipline(std::string discipline) const {
  std::transform(discipline.begin(), discipline.end(), discipline.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return discipline == "skidpad" ? skidpad_cone_penalty_time_s_ : default_cone_penalty_time_s_;
}

std::filesystem::path Statistics::run_directory() const {
  return std::filesystem::current_path() / "src" / "invictasim" / "runs";
}

void Statistics::reset() {
  snapshot_ = StatisticsSnapshot();
  previous_position_.reset();
  previous_start_line_side_.reset();
  lap_timing_started_ = false;
  lap_start_time_ = 0.0;
  lap_start_distance_ = 0.0;
  distance_traveled_ = 0.0;
  reset_lap_accumulators();
  reset_lap_cone_hits();
  start_csv_file();
}

void Statistics::update(const VehicleModelSnapshot& vehicle_snapshot, double sim_time,
                        double sim_dt,
                        const std::vector<common_lib::structures::PathPoint>& path_points) {
  const common_lib::structures::Position position(vehicle_snapshot.x, vehicle_snapshot.y);
  snapshot_.current_velocity = speed_from_snapshot(vehicle_snapshot);
  snapshot_.current_lap_time = lap_timing_started_ ? sim_time - lap_start_time_ : 0.0;

  if (previous_position_) {
    distance_traveled_ += distance_between(*previous_position_, position);
  }
  previous_position_ = position;

  update_lap_aggregates(snapshot_.current_velocity, sim_dt);
  update_tracking_metrics(vehicle_snapshot, path_points, sim_dt);
  update_cone_collisions(vehicle_snapshot);
  update_lap_counter(position, sim_time);
}

void Statistics::update_lap_aggregates(double speed, double sim_dt) {
  if (!lap_timing_started_) {
    return;
  }

  const double safe_dt = std::max(0.0, sim_dt);
  lap_accumulated_time_ += safe_dt;
  lap_velocity_integral_ += speed * safe_dt;

  lap_max_velocity_ = std::max(lap_max_velocity_, speed);
}

void Statistics::update_tracking_metrics(
    const VehicleModelSnapshot& vehicle_snapshot,
    const std::vector<common_lib::structures::PathPoint>& path_points, double sim_dt) {
  if (path_points.empty()) {
    snapshot_.objective_velocity = 0.0;
    snapshot_.tracking_cross_track_error = 0.0;
    return;
  }

  const common_lib::structures::Position car_position(vehicle_snapshot.x, vehicle_snapshot.y);
  const auto [cross_track_error, objective_velocity] =
      tracking_error_and_objective_velocity(car_position, path_points);

  snapshot_.objective_velocity = objective_velocity;
  snapshot_.tracking_cross_track_error = cross_track_error;
  snapshot_.velocity_error = objective_velocity - snapshot_.current_velocity;

  if (!lap_timing_started_) {
    return;
  }

  const double safe_dt = std::max(0.0, sim_dt);
  const double velocity_error = std::abs(snapshot_.current_velocity - objective_velocity);
  lap_tracking_accumulated_time_ += safe_dt;
  lap_tracking_error_integral_ += cross_track_error * safe_dt;
  lap_velocity_error_integral_ += velocity_error * safe_dt;
  lap_max_tracking_error_ = std::max(lap_max_tracking_error_, cross_track_error);
  lap_max_velocity_error_ = std::max(lap_max_velocity_error_, velocity_error);
}

double Statistics::cone_radius(const common_lib::structures::Cone& cone) const {
  return cone.is_large || cone.color == common_lib::competition_logic::Color::LARGE_ORANGE
             ? large_cone_radius_m_
             : standard_cone_radius_m_;
}

std::vector<Statistics::Hitbox> Statistics::load_car_hitboxes(
    const std::string& car_parameters_config) const {
  std::vector<Hitbox> hitboxes;
  if (car_parameters_config.empty()) {
    return hitboxes;
  }

  const std::string path = std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/car/" +
                           car_parameters_config + "/config.yaml";

  const YAML::Node config = YAML::LoadFile(path);
  const YAML::Node yaml_hitboxes = config["hitboxes"];
  if (!yaml_hitboxes) {
    return hitboxes;
  }
  const YAML::Node yaml_boxes = yaml_hitboxes["boxes"];
  if (!yaml_boxes) {
    return hitboxes;
  }
  if (!yaml_boxes.IsSequence()) {
    throw std::runtime_error("Malformed hitboxes. Expected a boxes list in: " + path);
  }

  for (const auto& node : yaml_boxes) {
    const double length = node["length"].as<double>();
    const double width = node["width"].as<double>();
    if (length <= 0.0 || width <= 0.0) {
      throw std::runtime_error("Malformed hitbox. length and width must be positive in: " + path);
    }

    hitboxes.push_back({node["center_x"].as<double>(0.0), node["center_y"].as<double>(0.0),
                        length * 0.5, width * 0.5});
  }

  return hitboxes;
}

bool Statistics::hitbox_collides_with_cone(const Hitbox& hitbox, double local_x, double local_y,
                                           double cone_radius) const {
  const double min_x = hitbox.center_x - hitbox.half_length;
  const double max_x = hitbox.center_x + hitbox.half_length;
  const double min_y = hitbox.center_y - hitbox.half_width;
  const double max_y = hitbox.center_y + hitbox.half_width;
  const double closest_x = std::clamp(local_x, min_x, max_x);
  const double closest_y = std::clamp(local_y, min_y, max_y);
  return std::hypot(local_x - closest_x, local_y - closest_y) <= cone_radius;
}

bool Statistics::collides_with_cone(const VehicleModelSnapshot& vehicle_snapshot,
                                    const common_lib::structures::Cone& cone) const {
  const double dx = cone.position.x - vehicle_snapshot.x;
  const double dy = cone.position.y - vehicle_snapshot.y;
  const double cos_yaw = std::cos(vehicle_snapshot.yaw);
  const double sin_yaw = std::sin(vehicle_snapshot.yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  const double radius = cone_radius(cone);
  for (const Hitbox& hitbox : car_hitboxes_) {
    if (hitbox_collides_with_cone(hitbox, local_x, local_y, radius)) {
      return true;
    }
  }

  return false;
}

double Statistics::current_lap_penalty_time() const {
  return static_cast<double>(hit_cone_indices_.size()) * penalty_time_per_cone_;
}

bool Statistics::cone_was_hit(std::size_t cone_index) const {
  return std::find(hit_cone_indices_.begin(), hit_cone_indices_.end(), cone_index) !=
         hit_cone_indices_.end();
}

double Statistics::current_lap_average_velocity() const {
  if (lap_accumulated_time_ <= 0.0) {
    return 0.0;
  }
  return lap_velocity_integral_ / lap_accumulated_time_;
}

double Statistics::current_lap_average_tracking_error() const {
  if (lap_tracking_accumulated_time_ <= 0.0) {
    return 0.0;
  }
  return lap_tracking_error_integral_ / lap_tracking_accumulated_time_;
}

double Statistics::current_lap_average_velocity_error() const {
  if (lap_tracking_accumulated_time_ <= 0.0) {
    return 0.0;
  }
  return lap_velocity_error_integral_ / lap_tracking_accumulated_time_;
}

void Statistics::update_cone_collisions(const VehicleModelSnapshot& vehicle_snapshot) {
  if (!lap_timing_started_) {
    return;
  }

  for (std::size_t i = 0; i < cones_.size(); ++i) {
    if (!collides_with_cone(vehicle_snapshot, cones_[i])) {
      continue;
    }

    if (cone_was_hit(i)) {
      continue;
    }

    hit_cone_indices_.push_back(i);
    snapshot_.current_lap_cones_hit = static_cast<int>(hit_cone_indices_.size());
  }
}

double Statistics::signed_start_line_distance(
    const common_lib::structures::Position& position) const {
  const double line_x = start_line_b_.x - start_line_a_.x;
  const double line_y = start_line_b_.y - start_line_a_.y;
  const double length = std::hypot(line_x, line_y);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }
  return (line_x * (position.y - start_line_a_.y) - line_y * (position.x - start_line_a_.x)) /
         length;
}

bool Statistics::is_inside_start_line_gate(const common_lib::structures::Position& position) const {
  const double line_x = start_line_b_.x - start_line_a_.x;
  const double line_y = start_line_b_.y - start_line_a_.y;
  const double length_sq = line_x * line_x + line_y * line_y;
  if (length_sq <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double t =
      ((position.x - start_line_a_.x) * line_x + (position.y - start_line_a_.y) * line_y) /
      length_sq;
  const double margin_ratio = start_line_gate_margin_m_ / std::sqrt(length_sq);
  return t >= -margin_ratio && t <= 1.0 + margin_ratio;
}

bool Statistics::crossed_start_line(const common_lib::structures::Position& position,
                                    double sim_time) const {
  if (!previous_start_line_side_ || !is_inside_start_line_gate(position)) {
    return false;
  }

  const double current_side = signed_start_line_distance(position);
  const bool changed_side = (*previous_start_line_side_ <= 0.0 && current_side > 0.0) ||
                            (*previous_start_line_side_ >= 0.0 && current_side < 0.0);
  const bool enough_time = (sim_time - lap_start_time_) >= minimum_lap_time_s_;
  const bool enough_distance =
      (distance_traveled_ - lap_start_distance_) >= minimum_lap_distance_m_;
  return changed_side && enough_time && enough_distance;
}

void Statistics::finish_lap(double sim_time) {
  const double lap_time = sim_time - lap_start_time_;
  snapshot_.lap_counter += 1;
  snapshot_.last_lap_time = lap_time;
  snapshot_.cones_hit = snapshot_.current_lap_cones_hit;
  snapshot_.total_lap_time = snapshot_.last_lap_time + current_lap_penalty_time();
  snapshot_.completed_lap_average_velocity = current_lap_average_velocity();
  snapshot_.completed_lap_max_velocity = lap_max_velocity_;
  snapshot_.completed_lap_average_tracking_error = current_lap_average_tracking_error();
  snapshot_.completed_lap_max_tracking_error = lap_max_tracking_error_;
  snapshot_.completed_lap_average_velocity_error = current_lap_average_velocity_error();
  snapshot_.completed_lap_max_velocity_error = lap_max_velocity_error_;

  if (snapshot_.best_lap_time <= 0.0 || snapshot_.total_lap_time < snapshot_.best_lap_time) {
    snapshot_.best_lap_time = snapshot_.total_lap_time;
  }

  write_csv_row();

  lap_start_time_ = sim_time;
  lap_start_distance_ = distance_traveled_;
  snapshot_.current_lap_time = 0.0;
  reset_lap_accumulators();
  reset_lap_cone_hits();
}

void Statistics::update_lap_counter(const common_lib::structures::Position& position,
                                    double sim_time) {
  const double current_side = signed_start_line_distance(position);
  const bool crossed_line = previous_start_line_side_ && is_inside_start_line_gate(position) &&
                            ((*previous_start_line_side_ <= 0.0 && current_side > 0.0) ||
                             (*previous_start_line_side_ >= 0.0 && current_side < 0.0));

  if (!lap_timing_started_) {
    if (crossed_line) {
      lap_timing_started_ = true;
      lap_start_time_ = sim_time;
      lap_start_distance_ = distance_traveled_;
      snapshot_.current_lap_time = 0.0;
      reset_lap_accumulators();
      reset_lap_cone_hits();
    }
    previous_start_line_side_ = current_side;
    return;
  }

  if (crossed_start_line(position, sim_time)) {
    finish_lap(sim_time);
  }

  previous_start_line_side_ = current_side;
}

void Statistics::reset_lap_accumulators() {
  lap_velocity_integral_ = 0.0;
  lap_accumulated_time_ = 0.0;
  lap_max_velocity_ = 0.0;
  lap_tracking_error_integral_ = 0.0;
  lap_max_tracking_error_ = 0.0;
  lap_velocity_error_integral_ = 0.0;
  lap_max_velocity_error_ = 0.0;
  lap_tracking_accumulated_time_ = 0.0;
}

void Statistics::reset_lap_cone_hits() {
  snapshot_.current_lap_cones_hit = 0;
  hit_cone_indices_.clear();
}

void Statistics::start_csv_file() {
  if (csv_file_.is_open()) {
    csv_file_.close();
  }

  std::error_code error;
  const auto directory = run_directory();
  std::filesystem::create_directories(directory, error);
  if (error) {
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  const auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  std::tm now_tm{};
#if defined(_WIN32)
  localtime_s(&now_tm, &now_time);
#else
  localtime_r(&now_time, &now_tm);
#endif

  std::ostringstream filename;
  filename << "invictasim_" << std::put_time(&now_tm, "%Y%m%d_%H%M%S") << "_" << std::setw(3)
           << std::setfill('0') << millis.count() << ".csv";
  csv_file_.open(directory / filename.str(), std::ios::out | std::ios::trunc);
  if (csv_file_.is_open()) {
    write_csv_header();
  }
}

void Statistics::write_csv_header() {
  csv_file_ << "lap_counter,lap_time,cones_hit,total_lap_time,best_lap_time,"
               "avg_velocity_kmh,max_velocity_kmh,"
               "avg_tracking_error_distance,max_tracking_error_distance,"
               "avg_velocity_error_kmh,max_velocity_error_kmh\n";
}

void Statistics::write_csv_row() {
  if (!csv_file_.is_open()) {
    return;
  }

  csv_file_ << snapshot_.lap_counter << "," << snapshot_.last_lap_time << "," << snapshot_.cones_hit
            << "," << snapshot_.total_lap_time << "," << snapshot_.best_lap_time << ","
            << snapshot_.completed_lap_average_velocity * meters_per_second_to_kilometers_per_hour_
            << ","
            << snapshot_.completed_lap_max_velocity * meters_per_second_to_kilometers_per_hour_
            << "," << snapshot_.completed_lap_average_tracking_error << ","
            << snapshot_.completed_lap_max_tracking_error << ","
            << snapshot_.completed_lap_average_velocity_error *
                   meters_per_second_to_kilometers_per_hour_
            << ","
            << snapshot_.completed_lap_max_velocity_error *
                   meters_per_second_to_kilometers_per_hour_
            << "\n";
  csv_file_.flush();
}
