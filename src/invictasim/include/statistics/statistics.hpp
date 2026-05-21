#pragma once

#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/position.hpp"
#include "io/output/output_snapshot.hpp"
#include "track/track.hpp"

/**
 * @brief Simulator statistics calculator.
 */
class Statistics {
public:
  /**
   * @brief Construct a statistics calculator using track metadata.
   * @param track Track definition used for lap line.
   */
  Statistics(const Track& track,
             std::shared_ptr<const common_lib::car_parameters::CarParameters> car_parameters,
             const std::string& discipline);

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
  void update(const VehicleModelSnapshot& snapshot, double sim_time, double sim_dt,
              const std::vector<common_lib::structures::PathPoint>& path_points = {});

  /**
   * @brief Get latest statistics snapshot.
   */
  StatisticsSnapshot get_snapshot() const { return snapshot_; }

  /**
   * @brief Get cones hit during the current lap for map visualization.
   */
  std::vector<common_lib::structures::Cone> get_recently_hit_cones() const {
    return recently_hit_cones_;
  }

private:
  static constexpr double start_line_gate_margin_m_ = 2.0;
  static constexpr double minimum_lap_time_s_ = 3.0;
  static constexpr double minimum_lap_distance_m_ = 20.0;
  static constexpr double meters_per_second_to_kilometers_per_hour_ = 3.6;
  static constexpr double standard_cone_radius_m_ = 0.115;
  static constexpr double large_cone_radius_m_ = 0.15;
  static constexpr double default_car_front_extent_m_ = 1.0;
  static constexpr double default_car_rear_extent_m_ = 1.0;
  static constexpr double default_car_half_width_m_ = 0.75;
  static constexpr double default_cone_penalty_time_s_ = 2.0;
  static constexpr double skidpad_cone_penalty_time_s_ = 0.2;

  common_lib::structures::Position start_line_a_;
  common_lib::structures::Position start_line_b_;
  std::vector<common_lib::structures::Cone> cones_;
  StatisticsSnapshot snapshot_;
  std::optional<common_lib::structures::Position> previous_position_;
  std::optional<double> previous_start_line_side_;
  bool lap_timing_started_ = false;
  double lap_start_time_ = 0.0;
  double lap_start_distance_ = 0.0;
  double lap_velocity_integral_ = 0.0;
  double lap_accumulated_time_ = 0.0;
  double lap_max_velocity_ = 0.0;
  double lap_tracking_error_integral_ = 0.0;
  double lap_max_tracking_error_ = 0.0;
  double lap_velocity_error_integral_ = 0.0;
  double lap_max_velocity_error_ = 0.0;
  double lap_tracking_accumulated_time_ = 0.0;
  double distance_traveled_ = 0.0;
  std::ofstream csv_file_;
  // Track which cones have been hit during the current lap
  std::vector<bool> cones_been_hit_;
  std::vector<common_lib::structures::Cone> recently_hit_cones_;
  double current_lap_penalty_time_ = 0.0;
  double penalty_time_per_cone_ = 0.0;
  double car_front_extent_ = 0.0;
  double car_rear_extent_ = 0.0;
  double car_half_width_ = 0.0;

  double distance_between(const common_lib::structures::Position& a,
                          const common_lib::structures::Position& b) const;
  std::pair<double, double> tracking_error_and_objective_velocity(
      const common_lib::structures::Position& car_position,
      const std::vector<common_lib::structures::PathPoint>& path_points) const;
  double speed_from_snapshot(const VehicleModelSnapshot& snapshot) const;
  double penalty_time_for_discipline(std::string discipline) const;
  std::filesystem::path run_directory() const;
  double signed_start_line_distance(const common_lib::structures::Position& position) const;
  bool is_inside_start_line_gate(const common_lib::structures::Position& position) const;
  bool crossed_start_line(const common_lib::structures::Position& position, double sim_time) const;
  bool collides_with_cone(const VehicleModelSnapshot& vehicle_snapshot,
                          const common_lib::structures::Cone& cone) const;
  double cone_radius(const common_lib::structures::Cone& cone) const;
  double current_lap_average_velocity() const;
  double current_lap_average_tracking_error() const;
  double current_lap_average_velocity_error() const;
  void update_lap_aggregates(double speed, double sim_dt);
  void update_cone_collisions(const VehicleModelSnapshot& vehicle_snapshot);
  void update_tracking_metrics(const VehicleModelSnapshot& vehicle_snapshot,
                               const std::vector<common_lib::structures::PathPoint>& path_points,
                               double sim_dt);
  void finish_lap(double sim_time);
  void reset_lap_cone_hits();
  void update_lap_counter(const common_lib::structures::Position& position, double sim_time);
  void reset_lap_accumulators();
  void start_csv_file();
  void write_csv_header();
  void write_csv_row(const char* event, double sim_time, double penalty_time);
};
