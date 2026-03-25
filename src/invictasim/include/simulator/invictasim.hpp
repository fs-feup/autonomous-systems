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

  /**
   * @brief Get the simulator configuration parameters. Used by adapters to get car config.
   */
  const InvictaSimParameters& get_params() const { return params_; }

  /**
   * @brief Get the complete vehicle model snapshot.
   * @return VehicleModelSnapshot Latest snapshot with all vehicle data.
   */
  VehicleModelSnapshot get_vehicle_model_snapshot() const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    return vehicle_model_snapshot_;
  }

  /**
   * @brief Get the latest execution times snapshot.
   * @return ExecutionTimesSnapshot Latest snapshot with execution timings.
   */
  ExecutionTimesSnapshot get_execution_times_snapshot() const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    return execution_times_snapshot_;
  }

private:
  InvictaSimParameters params_;  ///< Simulator configuration values.

  // Simulation components
  std::shared_ptr<VehicleModel> vehicle_model_;  ///< Vehicle model.

  // Simulation loop timing
  std::atomic<bool> running_;  ///< Indicates whether the simulation loop is running.
  double sim_time_;            ///< Current simulation time in seconds.
  std::chrono::steady_clock::duration step_duration_;  ///< Duration of each simulation step.
  std::chrono::steady_clock::time_point
      next_step_time_;  ///< Next wall-clock time for a simulation step.
  std::chrono::steady_clock::time_point
      last_step_time_;  ///< Last wall-clock time used to compute step dt.

  // Output snapshot data
  mutable std::mutex output_snapshot_mutex_;     ///< Protects output snapshot access.
  VehicleModelSnapshot vehicle_model_snapshot_;  ///< Latest consolidated vehicle model snapshot.
  ExecutionTimesSnapshot execution_times_snapshot_;  ///< Latest per-step execution timings.

  // Current commands
  mutable std::mutex input_mutex_;           ///< Protects input access.
  common_lib::structures::Wheels throttle_;  ///< Current throttle commands (all wheels).
  double steering_;                          ///< Current steering command (radians).

  /**
   * @brief Build a consolidated vehicle model snapshot with all vehicle state data.
   * @return VehicleModelSnapshot Consolidated snapshot with all vehicle data.
   */
  VehicleModelSnapshot build_vehicle_model_snapshot() const;

  /**
   * @brief Build per-subsystem execution timings snapshot.
   * @param total_step_ms Simulator step execution time in milliseconds.
   * @return ExecutionTimesSnapshot Timing snapshot.
   */
  ExecutionTimesSnapshot build_execution_times_snapshot(double total_step_ms) const;

  /**
   * @brief Get snapshots of current input.
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
