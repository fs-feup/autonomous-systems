#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "common_lib/structures/position.hpp"
#include "io/output/output_snapshot.hpp"
#include "track/track.hpp"

/**
 * @brief Result of checking the car against the lap timing line.
 */
enum class LapEvent { none, started, finished };

/**
 * @brief Tracks lap timing and velocity aggregates.
 */
class LapTimerStatistics {
public:
  /**
   * @brief Initialize lap timing from the track and discipline.
   */
  LapTimerStatistics(const Track& track, const std::string& discipline);

  /**
   * @brief Reset timing state and current lap fields.
   */
  void reset(StatisticsSnapshot& snapshot);

  /**
   * @brief Update lap timing and report whether a lap started or finished.
   */
  LapEvent update(const common_lib::structures::Position& position, double sim_time, double sim_dt,
                  double speed, StatisticsSnapshot& snapshot);

  /**
   * @brief Copy completed lap timing metrics into the snapshot.
   */
  void complete_lap(double sim_time, StatisticsSnapshot& snapshot) const;

  /**
   * @brief Start measuring the next lap after a finish.
   */
  void start_next_lap(double sim_time, StatisticsSnapshot& snapshot);

  /**
   * @brief Whether timing has started for the current event.
   */
  bool timing_started() const { return lap_timing_started_; }

private:
  // Timing line: pair of positions representing the endpoints of the line segment.
  using TimingLine = std::tuple<common_lib::structures::Position, common_lib::structures::Position>;

  // Timing line geometry.
  TimingLine timing_line_;
  TimingLine finish_line_;
  double timing_line_gate_margin_m_ = 2.0;
  double timing_line_approach_side_ = 0.0;
  double finish_line_approach_side_ = 0.0;
  double minimum_lap_time_s_ = 3.0;
  std::string discipline_;

  // Current event timing state.
  std::optional<double> previous_timing_line_side_;
  std::optional<double> previous_finish_line_side_;
  bool lap_timing_started_ = false;
  bool acceleration_finished_ = false;
  double lap_start_time_ = 0.0;
  double lap_velocity_integral_ = 0.0;
  double lap_accumulated_time_ = 0.0;
  double lap_max_velocity_ = 0.0;

  // Configuration and lap aggregate helpers.
  void load_config();
  void configure_timing_lines(const Track& track, std::string discipline);
  void configure_default_lines(const Track& track);
  void configure_acceleration_lines(const Track& track);
  void update_lap_aggregates(double speed, double sim_dt);
  void reset_lap_accumulators();
  double current_lap_average_velocity() const;

  // Line crossing helpers.
  double signed_line_distance(const TimingLine& line,
                              const common_lib::structures::Position& position) const;
  bool is_inside_line_gate(const TimingLine& line,
                           const common_lib::structures::Position& position) const;
  bool crossed_line(const TimingLine& line, std::optional<double> previous_side,
                    const common_lib::structures::Position& position, double current_side) const;
  bool crossed_line_from_approach_side(const TimingLine& line, std::optional<double> previous_side,
                                       const common_lib::structures::Position& position,
                                       double current_side, double approach_side) const;
  LapEvent update_default_lap(const common_lib::structures::Position& position, double current_side,
                              double sim_time);
  LapEvent update_acceleration(const common_lib::structures::Position& position, double sim_time);
};
