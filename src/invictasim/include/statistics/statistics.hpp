#pragma once

#include <limits>
#include <optional>
#include <fstream>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/position.hpp"
#include "io/output/output_snapshot.hpp"
#include "track/track.hpp"

/**
 * @brief Stateful simulator statistics calculator.
 */
class Statistics {
public:
  /**
   * @brief Construct a statistics calculator using track metadata.
   * @param track Track definition used for lap line and cross-track approximation.
   */
  explicit Statistics(const Track& track);

  /**
   * @brief Reset all accumulated statistics.
   */
  void reset();

  /**
   * @brief Update statistics after a vehicle model step.
   * @param snapshot Latest vehicle model snapshot.
   * @param sim_time Current simulator time in seconds.
   * @param sim_dt Last simulation step duration in seconds.
   */
  void update(const VehicleModelSnapshot& snapshot, double sim_time, double sim_dt);

  /**
   * @brief Get latest statistics snapshot.
   */
  StatisticsSnapshot get_snapshot() const { return snapshot_; }

private:
  common_lib::structures::Position start_line_a_;
  common_lib::structures::Position start_line_b_;
  std::vector<common_lib::structures::Cone> cones_;
  StatisticsSnapshot snapshot_;
  std::optional<common_lib::structures::Position> previous_position_;
  std::optional<double> previous_start_line_side_;
  double lap_start_time_ = 0.0;
  double lap_start_distance_ = 0.0;
  double lap_velocity_integral_ = 0.0;
  double lap_cross_track_error_integral_ = 0.0;
  double lap_accumulated_time_ = 0.0;
  double lap_max_velocity_ = 0.0;
  double lap_max_cross_track_error_ = 0.0;
  double lap_max_longitudinal_acceleration_ = 0.0;
  double lap_max_lateral_acceleration_ = 0.0;
  double lap_max_yaw_rate_ = 0.0;
  std::ofstream csv_file_;

  double signed_start_line_distance(const common_lib::structures::Position& position) const;
  bool is_inside_start_line_gate(const common_lib::structures::Position& position) const;
  bool crossed_start_line(const common_lib::structures::Position& position, double sim_time) const;
  double calculate_cross_track_error(const common_lib::structures::Position& position) const;
  void update_lap_counter(const common_lib::structures::Position& position, double sim_time);
  void reset_lap_accumulators();
  void start_csv_file();
  void write_csv_header();
  void write_csv_row(const char* event);
};
