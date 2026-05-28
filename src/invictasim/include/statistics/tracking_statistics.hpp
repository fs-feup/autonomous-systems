#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/position.hpp"
#include "io/output/output_snapshot.hpp"

/**
 * @brief Tracks path following error and objective velocity metrics.
 */
class TrackingStatistics {
public:
  /**
   * @brief Clear per-lap tracking aggregates.
   */
  void reset_lap();

  /**
   * @brief Update tracking error from the current vehicle pose and path.
   */
  void update(const VehicleModelSnapshot& vehicle_snapshot,
              const std::vector<common_lib::structures::PathPoint>& path_points, double sim_dt,
              bool lap_timing_started, StatisticsSnapshot& snapshot);

  /**
   * @brief Copy completed lap tracking metrics into the snapshot.
   */
  void complete_lap(StatisticsSnapshot& snapshot) const;

private:
  // Current lap tracking aggregates.
  double lap_tracking_error_integral_ = 0.0;
  double lap_max_tracking_error_ = 0.0;
  double lap_velocity_error_integral_ = 0.0;
  double lap_max_velocity_error_ = 0.0;
  double lap_tracking_accumulated_time_ = 0.0;

  // Path projection helpers.
  double distance_between(const common_lib::structures::Position& a,
                          const common_lib::structures::Position& b) const;
  std::pair<double, double> tracking_error_and_objective_velocity(
      const common_lib::structures::Position& car_position,
      const std::vector<common_lib::structures::PathPoint>& path_points) const;
  double current_lap_average_tracking_error() const;
  double current_lap_average_velocity_error() const;
};
