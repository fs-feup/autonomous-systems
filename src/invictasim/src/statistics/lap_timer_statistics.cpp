#include "statistics/lap_timer_statistics.hpp"

LapTimerStatistics::LapTimerStatistics(const Track& track, const std::string& discipline) {
  load_config();
  configure_timing_lines(track, discipline);
}

void LapTimerStatistics::load_config() {
  const std::string path =
      std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/ground/config.yaml";
  const YAML::Node config = YAML::LoadFile(path);
  const YAML::Node lap_finish = config["lap_finish"];
  if (!lap_finish) {
    return;
  }

  timing_line_gate_margin_m_ =
      lap_finish["timing_line_gate_margin"].as<double>(timing_line_gate_margin_m_);
  minimum_lap_time_s_ = lap_finish["minimum_lap_time"].as<double>(minimum_lap_time_s_);
}

void LapTimerStatistics::configure_timing_lines(const Track& track, std::string discipline) {
  std::transform(discipline.begin(), discipline.end(), discipline.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  discipline_ = discipline;

  if (discipline_ == "acceleration") {
    configure_acceleration_lines(track);
  } else {
    configure_default_lines(track);
  }
}

void LapTimerStatistics::configure_default_lines(const Track& track) {
  timing_line_ = track.get_timing_line();
  finish_line_ = timing_line_;
  timing_line_approach_side_ = signed_line_distance(timing_line_, track.get_start_position());
  finish_line_approach_side_ = timing_line_approach_side_;
}

void LapTimerStatistics::configure_acceleration_lines(const Track& track) {
  const auto& timing_lines = track.get_timing_lines();
  if (timing_lines.size() >= 2) {
    timing_line_ = {timing_lines[0].first, timing_lines[0].second};
    finish_line_ = {timing_lines[1].first, timing_lines[1].second};
    timing_line_approach_side_ = signed_line_distance(timing_line_, track.get_start_position());
    finish_line_approach_side_ = signed_line_distance(finish_line_, track.get_start_position());
    return;
  }

  throw std::runtime_error(
      "Acceleration track requires two timing_lines: timing gate and 75 m gate");
}

void LapTimerStatistics::reset(StatisticsSnapshot& snapshot) {
  previous_timing_line_side_.reset();
  previous_finish_line_side_.reset();
  lap_timing_started_ = false;
  acceleration_finished_ = false;
  lap_start_time_ = 0.0;
  snapshot.current_lap_time = 0.0;
  reset_lap_accumulators();
}

LapEvent LapTimerStatistics::update(const common_lib::structures::Position& position,
                                    double sim_time, double sim_dt, double speed,
                                    StatisticsSnapshot& snapshot) {
  snapshot.current_lap_time = lap_timing_started_ ? sim_time - lap_start_time_ : 0.0;

  update_lap_aggregates(speed, sim_dt);

  const double current_side = signed_line_distance(timing_line_, position);
  if (discipline_ == "acceleration") {
    return update_acceleration(position, sim_time);
  }
  return update_default_lap(position, current_side, sim_time);
}

LapEvent LapTimerStatistics::update_default_lap(const common_lib::structures::Position& position,
                                                double current_side, double sim_time) {
  const bool line_crossed = crossed_line_from_approach_side(
      timing_line_, previous_timing_line_side_, position, current_side, timing_line_approach_side_);

  if (!lap_timing_started_) {
    if (line_crossed) {
      lap_timing_started_ = true;
      lap_start_time_ = sim_time;
      reset_lap_accumulators();
      previous_timing_line_side_ = current_side;
      return LapEvent::started;
    }
    previous_timing_line_side_ = current_side;
    return LapEvent::none;
  }

  if (line_crossed && (sim_time - lap_start_time_) >= minimum_lap_time_s_) {
    previous_timing_line_side_ = current_side;
    return LapEvent::finished;
  }

  previous_timing_line_side_ = current_side;
  return LapEvent::none;
}

LapEvent LapTimerStatistics::update_acceleration(const common_lib::structures::Position& position,
                                                 double sim_time) {
  if (acceleration_finished_) {
    previous_timing_line_side_ = signed_line_distance(timing_line_, position);
    previous_finish_line_side_ = signed_line_distance(finish_line_, position);
    return LapEvent::none;
  }

  const double timing_side = signed_line_distance(timing_line_, position);
  const double finish_side = signed_line_distance(finish_line_, position);

  if (!lap_timing_started_) {
    if (crossed_line_from_approach_side(timing_line_, previous_timing_line_side_, position,
                                        timing_side, timing_line_approach_side_)) {
      lap_timing_started_ = true;
      lap_start_time_ = sim_time;
      reset_lap_accumulators();
      previous_timing_line_side_ = timing_side;
      previous_finish_line_side_ = finish_side;
      return LapEvent::started;
    }
    previous_timing_line_side_ = timing_side;
    previous_finish_line_side_ = finish_side;
    return LapEvent::none;
  }

  if (crossed_line_from_approach_side(finish_line_, previous_finish_line_side_, position,
                                      finish_side, finish_line_approach_side_)) {
    acceleration_finished_ = true;
    previous_timing_line_side_ = timing_side;
    previous_finish_line_side_ = finish_side;
    return LapEvent::finished;
  }

  previous_timing_line_side_ = timing_side;
  previous_finish_line_side_ = finish_side;
  return LapEvent::none;
}

void LapTimerStatistics::complete_lap(double sim_time, StatisticsSnapshot& snapshot) const {
  snapshot.lap_counter += 1;
  snapshot.last_lap_time = sim_time - lap_start_time_;
  snapshot.completed_lap_average_velocity = current_lap_average_velocity();
  snapshot.completed_lap_max_velocity = lap_max_velocity_;
}

void LapTimerStatistics::start_next_lap(double sim_time, StatisticsSnapshot& snapshot) {
  lap_start_time_ = sim_time;
  snapshot.current_lap_time = 0.0;
  reset_lap_accumulators();
}

void LapTimerStatistics::update_lap_aggregates(double speed, double sim_dt) {
  if (!lap_timing_started_) {
    return;
  }

  const double safe_dt = std::max(0.0, sim_dt);
  lap_accumulated_time_ += safe_dt;
  lap_velocity_integral_ += speed * safe_dt;
  lap_max_velocity_ = std::max(lap_max_velocity_, speed);
}

void LapTimerStatistics::reset_lap_accumulators() {
  lap_velocity_integral_ = 0.0;
  lap_accumulated_time_ = 0.0;
  lap_max_velocity_ = 0.0;
}

double LapTimerStatistics::current_lap_average_velocity() const {
  if (lap_accumulated_time_ <= 0.0) {
    return 0.0;
  }
  return lap_velocity_integral_ / lap_accumulated_time_;
}

double LapTimerStatistics::signed_line_distance(
    const TimingLine& line, const common_lib::structures::Position& position) const {
  const double line_x = std::get<1>(line).x - std::get<0>(line).x;
  const double line_y = std::get<1>(line).y - std::get<0>(line).y;
  const double length = std::hypot(line_x, line_y);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }
  return (line_x * (position.y - std::get<0>(line).y) -
          line_y * (position.x - std::get<0>(line).x)) /
         length;
}

bool LapTimerStatistics::is_inside_line_gate(
    const TimingLine& line, const common_lib::structures::Position& position) const {
  const double line_x = std::get<1>(line).x - std::get<0>(line).x;
  const double line_y = std::get<1>(line).y - std::get<0>(line).y;
  const double length_sq = line_x * line_x + line_y * line_y;
  if (length_sq <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double t =
      ((position.x - std::get<0>(line).x) * line_x + (position.y - std::get<0>(line).y) * line_y) /
      length_sq;
  const double margin_ratio = timing_line_gate_margin_m_ / std::sqrt(length_sq);
  return t >= -margin_ratio && t <= 1.0 + margin_ratio;
}

bool LapTimerStatistics::crossed_line(const TimingLine& line, std::optional<double> previous_side,
                                      const common_lib::structures::Position& position,
                                      double current_side) const {
  if (!previous_side || !is_inside_line_gate(line, position)) {
    return false;
  }

  return (*previous_side <= 0.0 && current_side > 0.0) ||
         (*previous_side >= 0.0 && current_side < 0.0);
}

bool LapTimerStatistics::crossed_line_from_approach_side(
    const TimingLine& line, std::optional<double> previous_side,
    const common_lib::structures::Position& position, double current_side,
    double approach_side) const {
  if (!previous_side || !is_inside_line_gate(line, position)) {
    return false;
  }

  if (std::abs(approach_side) <= std::numeric_limits<double>::epsilon()) {
    return crossed_line(line, previous_side, position, current_side);
  }

  return approach_side > 0.0 ? *previous_side >= 0.0 && current_side < 0.0
                             : *previous_side <= 0.0 && current_side > 0.0;
}
