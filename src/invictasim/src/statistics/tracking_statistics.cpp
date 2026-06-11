#include "statistics/tracking_statistics.hpp"

void TrackingStatistics::reset_lap() {
  lap_tracking_error_integral_ = 0.0;
  lap_max_tracking_error_ = 0.0;
  lap_velocity_error_integral_ = 0.0;
  lap_max_velocity_error_ = 0.0;
  lap_tracking_accumulated_time_ = 0.0;
}

void TrackingStatistics::update(const VehicleModelSnapshot& vehicle_snapshot,
                                const std::vector<common_lib::structures::PathPoint>& path_points,
                                double sim_dt, bool lap_timing_started,
                                StatisticsSnapshot& snapshot) {
  if (path_points.empty()) {
    snapshot.objective_velocity = 0.0;
    snapshot.tracking_cross_track_error = 0.0;
    return;
  }

  const common_lib::structures::Position car_position(vehicle_snapshot.x, vehicle_snapshot.y);
  const auto [cross_track_error, objective_velocity] =
      tracking_error_and_objective_velocity(car_position, path_points);

  snapshot.objective_velocity = objective_velocity;
  snapshot.tracking_cross_track_error = cross_track_error;
  snapshot.velocity_error = objective_velocity - snapshot.current_velocity;

  if (!lap_timing_started) {
    return;
  }

  const double safe_dt = std::max(0.0, sim_dt);
  const double velocity_error = std::abs(snapshot.current_velocity - objective_velocity);
  lap_tracking_accumulated_time_ += safe_dt;
  lap_tracking_error_integral_ += cross_track_error * safe_dt;
  lap_velocity_error_integral_ += velocity_error * safe_dt;
  lap_max_tracking_error_ = std::max(lap_max_tracking_error_, cross_track_error);
  lap_max_velocity_error_ = std::max(lap_max_velocity_error_, velocity_error);
}

void TrackingStatistics::complete_lap(StatisticsSnapshot& snapshot) const {
  snapshot.completed_lap_average_tracking_error = current_lap_average_tracking_error();
  snapshot.completed_lap_max_tracking_error = lap_max_tracking_error_;
  snapshot.completed_lap_average_velocity_error = current_lap_average_velocity_error();
  snapshot.completed_lap_max_velocity_error = lap_max_velocity_error_;
}

double TrackingStatistics::distance_between(const common_lib::structures::Position& a,
                                            const common_lib::structures::Position& b) const {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

std::pair<double, double> TrackingStatistics::tracking_error_and_objective_velocity(
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

double TrackingStatistics::current_lap_average_tracking_error() const {
  if (lap_tracking_accumulated_time_ <= 0.0) {
    return 0.0;
  }
  return lap_tracking_error_integral_ / lap_tracking_accumulated_time_;
}

double TrackingStatistics::current_lap_average_velocity_error() const {
  if (lap_tracking_accumulated_time_ <= 0.0) {
    return 0.0;
  }
  return lap_velocity_error_integral_ / lap_tracking_accumulated_time_;
}
