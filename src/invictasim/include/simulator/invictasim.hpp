#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

#include "common_lib/structures/wheels.hpp"
#include "config/config.hpp"
#include "io/output/output_snapshot.hpp"
#include "vehicle_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Main simulator class
 */
class InvictaSim {
public:
  /**
   * @brief Construct a new InvictaSim.
   * @param params Simulator configuration.
   */
  explicit InvictaSim(const InvictaSimParameters& params);

  /**
   * @brief Destroy the InvictaSim.
   */
  ~InvictaSim() = default;

  /**
   * @brief Start the simulation loop.
   */
  void run();

  /**
   * @brief Stop the simulation loop.
   */
  void stop();

  /**
   * @brief Set steering and throttle input.
   * @param throttle Throttle commands for all wheels.
   * @param steering Steering command (radians).
   */
  void set_input(const common_lib::structures::Wheels& throttle, double steering) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    throttle_ = throttle;
    steering_ = steering;
  }

  const InvictaSimParameters& get_params() const { return params_; }

  AggregateOutputSnapshot get_output_snapshot() const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    return latest_output_snapshot_;
  }

  void get_tire_snapshot(TireSnapshot& out_snapshot) const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    out_snapshot = latest_output_snapshot_.tire;
  }

  void get_powertrain_snapshot(PowertrainSnapshot& out_snapshot) const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    out_snapshot = latest_output_snapshot_.powertrain;
  }

  void get_aero_snapshot(AeroSnapshot& out_snapshot) const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    out_snapshot = latest_output_snapshot_.aero;
  }

  void get_load_snapshot(LoadSnapshot& out_snapshot) const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    out_snapshot = latest_output_snapshot_.load;
  }

  void get_status_snapshot(StatusSnapshot& out_snapshot) const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    out_snapshot = latest_output_snapshot_.status;
  }

private:
  InvictaSimParameters params_;                  ///< Simulator configuration values.
  std::shared_ptr<VehicleModel> vehicle_model_;  ///< Vehicle model.
  std::atomic<bool> running_;  ///< Indicates whether the simulation loop is running.
  double sim_time_;            ///< Current simulation time in seconds.
  std::chrono::steady_clock::time_point
      next_loop_time_;                              ///< Next wall-clock time for a simulation step.
  mutable std::mutex input_mutex_;                  ///< Protects input access.
  mutable std::mutex output_snapshot_mutex_;        ///< Protects output snapshot access.
  common_lib::structures::Wheels throttle_;         ///< Current throttle commands (all wheels).
  double steering_;                                 ///< Current steering command (radians).
  AggregateOutputSnapshot latest_output_snapshot_;  ///< Latest simulator output snapshot.

  /**
   * @brief Get snapshots of current input (locks briefly, returns values).
   * @param out_throttle Output throttle snapshot.
   * @param out_steering Output steering snapshot.
   */
  void get_input_snapshot(common_lib::structures::Wheels& out_throttle,
                          double& out_steering) const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    out_throttle = throttle_;
    out_steering = steering_;
  }

  /**
   * @brief Execute a simulation step.
   */
  void simulation_step();
};
