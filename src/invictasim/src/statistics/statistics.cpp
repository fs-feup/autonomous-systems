#include "statistics/statistics.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace {
constexpr double kStartLineGateMarginM = 2.0;
constexpr double kMinimumLapTimeS = 3.0;
constexpr double kMinimumLapDistanceM = 20.0;
constexpr double kMetersPerSecondToKilometersPerHour = 3.6;

double distance_between(const common_lib::structures::Position& a,
                        const common_lib::structures::Position& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double speed_from_snapshot(const VehicleModelSnapshot& snapshot) {
  return std::hypot(snapshot.velocity_x, snapshot.velocity_y);
}

bool is_blue_cone(const common_lib::structures::Cone& cone) {
  return cone.color == common_lib::competition_logic::Color::BLUE;
}

bool is_yellow_cone(const common_lib::structures::Cone& cone) {
  return cone.color == common_lib::competition_logic::Color::YELLOW;
}

std::filesystem::path run_directory() {
#ifdef INVICTASIM_SOURCE_DIR
  return std::filesystem::path(INVICTASIM_SOURCE_DIR) / "run";
#else
  return std::filesystem::current_path() / "src" / "invictasim" / "run";
#endif
}
}  // namespace

Statistics::Statistics(const Track& track)
    : start_line_a_(track.get_start_line().first),
      start_line_b_(track.get_start_line().second),
      cones_(track.get_cones()) {
  reset();
}

void Statistics::reset() {
  snapshot_ = StatisticsSnapshot();
  previous_position_.reset();
  previous_start_line_side_.reset();
  lap_start_time_ = 0.0;
  lap_start_distance_ = 0.0;
  reset_lap_accumulators();
  start_csv_file();
}

void Statistics::update(const VehicleModelSnapshot& vehicle_snapshot, double sim_time,
                        double sim_dt) {
  const common_lib::structures::Position position(vehicle_snapshot.x, vehicle_snapshot.y);
  const double speed = speed_from_snapshot(vehicle_snapshot);
  const double cross_track_error = calculate_cross_track_error(position);
  const double safe_dt = std::max(0.0, sim_dt);

  snapshot_.sim_time = sim_time;
  snapshot_.lap_completed = false;
  snapshot_.current_lap_time = sim_time - lap_start_time_;
  snapshot_.current_velocity = speed;
  snapshot_.current_cross_track_error = cross_track_error;
  snapshot_.current_longitudinal_acceleration = std::abs(vehicle_snapshot.acceleration_x);
  snapshot_.current_lateral_acceleration = std::abs(vehicle_snapshot.acceleration_y);
  snapshot_.current_yaw_rate = std::abs(vehicle_snapshot.yaw_rate);

  if (previous_position_) {
    snapshot_.distance_traveled += distance_between(*previous_position_, position);
  }
  previous_position_ = position;

  lap_accumulated_time_ += safe_dt;
  lap_velocity_integral_ += speed * safe_dt;
  lap_cross_track_error_integral_ += cross_track_error * safe_dt;
  if (lap_accumulated_time_ > 0.0) {
    snapshot_.average_velocity = lap_velocity_integral_ / lap_accumulated_time_;
    snapshot_.average_cross_track_error =
        lap_cross_track_error_integral_ / lap_accumulated_time_;
  }

  lap_max_velocity_ = std::max(lap_max_velocity_, speed);
  lap_max_cross_track_error_ = std::max(lap_max_cross_track_error_, cross_track_error);
  lap_max_longitudinal_acceleration_ =
      std::max(lap_max_longitudinal_acceleration_, snapshot_.current_longitudinal_acceleration);
  lap_max_lateral_acceleration_ =
      std::max(lap_max_lateral_acceleration_, snapshot_.current_lateral_acceleration);
  lap_max_yaw_rate_ = std::max(lap_max_yaw_rate_, snapshot_.current_yaw_rate);
  snapshot_.max_velocity = lap_max_velocity_;
  snapshot_.max_cross_track_error = lap_max_cross_track_error_;
  snapshot_.max_longitudinal_acceleration = lap_max_longitudinal_acceleration_;
  snapshot_.max_lateral_acceleration = lap_max_lateral_acceleration_;
  snapshot_.max_yaw_rate = lap_max_yaw_rate_;

  update_lap_counter(position, sim_time);
  if (snapshot_.lap_completed) {
    write_csv_row("lap_summary");
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
  const double margin_ratio = kStartLineGateMarginM / std::sqrt(length_sq);
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
  const bool enough_time = (sim_time - lap_start_time_) >= kMinimumLapTimeS;
  const bool enough_distance =
      (snapshot_.distance_traveled - lap_start_distance_) >= kMinimumLapDistanceM;
  return changed_side && enough_time && enough_distance;
}

double Statistics::calculate_cross_track_error(
    const common_lib::structures::Position& position) const {
  const common_lib::structures::Cone* nearest_blue = nullptr;
  const common_lib::structures::Cone* nearest_yellow = nullptr;
  double nearest_blue_distance = std::numeric_limits<double>::max();
  double nearest_yellow_distance = std::numeric_limits<double>::max();

  for (const auto& cone : cones_) {
    const double dx = position.x - cone.position.x;
    const double dy = position.y - cone.position.y;
    const double distance = dx * dx + dy * dy;
    if (is_blue_cone(cone) && distance < nearest_blue_distance) {
      nearest_blue = &cone;
      nearest_blue_distance = distance;
    } else if (is_yellow_cone(cone) && distance < nearest_yellow_distance) {
      nearest_yellow = &cone;
      nearest_yellow_distance = distance;
    }
  }

  if (nearest_blue == nullptr || nearest_yellow == nullptr) {
    return 0.0;
  }

  const common_lib::structures::Position centerline_point(
      0.5 * (nearest_blue->position.x + nearest_yellow->position.x),
      0.5 * (nearest_blue->position.y + nearest_yellow->position.y));
  return distance_between(position, centerline_point);
}

void Statistics::update_lap_counter(const common_lib::structures::Position& position,
                                    double sim_time) {
  if (crossed_start_line(position, sim_time)) {
    const double lap_time = sim_time - lap_start_time_;
    snapshot_.lap_counter += 1;
    snapshot_.lap_completed = true;
    snapshot_.last_lap_time = lap_time;
    snapshot_.completed_lap_average_velocity = snapshot_.average_velocity;
    snapshot_.completed_lap_max_velocity = snapshot_.max_velocity;
    snapshot_.completed_lap_average_cross_track_error = snapshot_.average_cross_track_error;
    snapshot_.completed_lap_max_cross_track_error = snapshot_.max_cross_track_error;
    snapshot_.completed_lap_max_longitudinal_acceleration =
        snapshot_.max_longitudinal_acceleration;
    snapshot_.completed_lap_max_lateral_acceleration = snapshot_.max_lateral_acceleration;
    snapshot_.completed_lap_max_yaw_rate = snapshot_.max_yaw_rate;
    if (snapshot_.best_lap_time <= 0.0 || lap_time < snapshot_.best_lap_time) {
      snapshot_.best_lap_time = lap_time;
    }
    lap_start_time_ = sim_time;
    lap_start_distance_ = snapshot_.distance_traveled;
    snapshot_.current_lap_time = 0.0;
    reset_lap_accumulators();
  }

  previous_start_line_side_ = signed_start_line_distance(position);
}

void Statistics::reset_lap_accumulators() {
  lap_velocity_integral_ = 0.0;
  lap_cross_track_error_integral_ = 0.0;
  lap_accumulated_time_ = 0.0;
  lap_max_velocity_ = 0.0;
  lap_max_cross_track_error_ = 0.0;
  lap_max_longitudinal_acceleration_ = 0.0;
  lap_max_lateral_acceleration_ = 0.0;
  lap_max_yaw_rate_ = 0.0;
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
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;
  std::tm now_tm{};
#if defined(_WIN32)
  localtime_s(&now_tm, &now_time);
#else
  localtime_r(&now_time, &now_tm);
#endif

  std::ostringstream filename;
  filename << "invictasim_" << std::put_time(&now_tm, "%Y%m%d_%H%M%S") << "_"
           << std::setw(3) << std::setfill('0') << millis.count() << ".csv";
  csv_file_.open(directory / filename.str(), std::ios::out | std::ios::trunc);
  if (csv_file_.is_open()) {
    write_csv_header();
  }
}

void Statistics::write_csv_header() {
  csv_file_ << "event,sim_time,lap_counter,lap_time,best_lap_time,distance_traveled,"
               "average_velocity_kmh,max_velocity_kmh,average_cross_track_error,"
               "max_cross_track_error,max_longitudinal_acceleration,max_lateral_acceleration,"
               "max_yaw_rate\n";
}

void Statistics::write_csv_row(const char* event) {
  if (!csv_file_.is_open()) {
    return;
  }

  csv_file_ << event << "," << snapshot_.sim_time << "," << snapshot_.lap_counter << ","
            << snapshot_.last_lap_time << "," << snapshot_.best_lap_time << ","
            << snapshot_.distance_traveled << ","
            << snapshot_.completed_lap_average_velocity *
                   kMetersPerSecondToKilometersPerHour
            << ","
            << snapshot_.completed_lap_max_velocity * kMetersPerSecondToKilometersPerHour << ","
            << snapshot_.completed_lap_average_cross_track_error << ","
            << snapshot_.completed_lap_max_cross_track_error << ","
            << snapshot_.completed_lap_max_longitudinal_acceleration << ","
            << snapshot_.completed_lap_max_lateral_acceleration << ","
            << snapshot_.completed_lap_max_yaw_rate << "\n";
  csv_file_.flush();
}
