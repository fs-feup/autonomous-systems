#pragma once

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "io/output/output_snapshot.hpp"
#include "statistics/collision_statistics.hpp"
#include "statistics/lap_timer_statistics.hpp"
#include "statistics/tracking_statistics.hpp"
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
             const std::string& discipline, const std::string& car_parameters_config = "");

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
  std::vector<common_lib::structures::Cone> get_recently_hit_cones() const;

private:
  StatisticsSnapshot snapshot_;
  CollisionStatistics collision_statistics_;
  LapTimerStatistics lap_timer_statistics_;
  TrackingStatistics tracking_statistics_;
  std::ofstream csv_file_;

  double speed_from_snapshot(const VehicleModelSnapshot& snapshot) const;
  void finish_lap(double sim_time);
  std::filesystem::path run_directory() const;
  void start_csv_file();
  void write_csv_header();
  void write_csv_row();
};
