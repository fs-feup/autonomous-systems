#include "statistics/statistics.hpp"

Statistics::Statistics(
    const Track& track,
    std::shared_ptr<const common_lib::car_parameters::CarParameters> car_parameters,
    const std::string& discipline, const std::string& car_parameters_config)
    : collision_statistics_(track, discipline, car_parameters_config),
      lap_timer_statistics_(track, discipline) {
  reset();
}

std::vector<common_lib::structures::Cone> Statistics::get_recently_hit_cones() const {
  return collision_statistics_.recently_hit_cones();
}

double Statistics::speed_from_snapshot(const VehicleModelSnapshot& snapshot) const {
  return std::hypot(snapshot.velocity_x, snapshot.velocity_y);
}

void Statistics::reset() {
  snapshot_ = StatisticsSnapshot();
  lap_timer_statistics_.reset(snapshot_);
  tracking_statistics_.reset_lap();
  collision_statistics_.reset_lap(snapshot_);
  start_csv_file();
}

void Statistics::update(const VehicleModelSnapshot& vehicle_snapshot, double sim_time,
                        double sim_dt,
                        const std::vector<common_lib::structures::PathPoint>& path_points) {
  const common_lib::structures::Position position(vehicle_snapshot.x, vehicle_snapshot.y);
  const bool lap_was_running = lap_timer_statistics_.timing_started();
  snapshot_.current_velocity = speed_from_snapshot(vehicle_snapshot);

  tracking_statistics_.update(vehicle_snapshot, path_points, sim_dt, lap_was_running, snapshot_);
  collision_statistics_.update(vehicle_snapshot, lap_was_running, snapshot_);

  const LapEvent lap_event = lap_timer_statistics_.update(position, sim_time, sim_dt,
                                                          snapshot_.current_velocity, snapshot_);
  if (lap_event == LapEvent::started) {
    tracking_statistics_.reset_lap();
    collision_statistics_.reset_lap(snapshot_);
  } else if (lap_event == LapEvent::finished) {
    finish_lap(sim_time);
  }
}

void Statistics::finish_lap(double sim_time) {
  lap_timer_statistics_.complete_lap(sim_time, snapshot_);
  tracking_statistics_.complete_lap(snapshot_);
  snapshot_.cones_hit = snapshot_.current_lap_cones_hit;
  snapshot_.total_lap_time =
      snapshot_.last_lap_time + collision_statistics_.current_lap_penalty_time();

  if (snapshot_.best_lap_time <= 0.0 || snapshot_.total_lap_time < snapshot_.best_lap_time) {
    snapshot_.best_lap_time = snapshot_.total_lap_time;
  }

  write_csv_row();
  lap_timer_statistics_.start_next_lap(sim_time, snapshot_);
  tracking_statistics_.reset_lap();
  collision_statistics_.reset_lap(snapshot_);
}

std::filesystem::path Statistics::run_directory() const {
  return std::filesystem::current_path() / "src" / "invictasim" / "runs";
}

void Statistics::start_csv_file() {
  if (csv_file_.is_open()) {
    csv_file_.close();
  }

  const auto directory = run_directory();
  std::filesystem::create_directories(directory);

  const auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
  const std::string filename = "invictasim_" + std::to_string(timestamp_ms) + ".csv";

  csv_file_.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  csv_file_.open(directory / filename, std::ios::out | std::ios::trunc);
  write_csv_header();
}

void Statistics::write_csv_header() {
  csv_file_ << "lap_counter,lap_time,cones_hit,total_lap_time,best_lap_time,"
               "avg_velocity_kmh,max_velocity_kmh,"
               "avg_tracking_error_distance,max_tracking_error_distance,"
               "avg_velocity_error_kmh,max_velocity_error_kmh\n";
}

void Statistics::write_csv_row() {
  csv_file_ << snapshot_.lap_counter << "," << snapshot_.last_lap_time << "," << snapshot_.cones_hit
            << "," << snapshot_.total_lap_time << "," << snapshot_.best_lap_time << ","
            << snapshot_.completed_lap_average_velocity * 3.6 << ","
            << snapshot_.completed_lap_max_velocity * 3.6 << ","
            << snapshot_.completed_lap_average_tracking_error << ","
            << snapshot_.completed_lap_max_tracking_error << ","
            << snapshot_.completed_lap_average_velocity_error * 3.6 << ","
            << snapshot_.completed_lap_max_velocity_error * 3.6 << "\n";
  csv_file_.flush();
}
