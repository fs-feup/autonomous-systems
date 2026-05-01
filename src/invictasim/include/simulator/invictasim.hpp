#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

#include "common_lib/structures/wheels.hpp"
#include "config/config.hpp"
#include "io/output/output_snapshot.hpp"
#include "track/track.hpp"
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
    if (!go_signal_) {
      throttle_ = {0.0, 0.0, 0.0, 0.0};
      steering_ = 0.0;
    } else {
      throttle_ = throttle;
      steering_ = steering;
    }
  }

  /**
   * @brief Activate the go signal state.
   */
  void activate_go_signal();

  /**
   * @brief Modify the simulation speed by adding a delta.
   * @param delta The value to add to the simulation speed.
   */
  void add_sim_speed(double delta);

  /**
   * @brief Toggle the Emergency Brake System.
   */
  void toggle_ebs();

  /**
   * @brief Reset the simulation to default state.
   */
  void reset_sim();

  /**
   * @brief Check if the simulator is running.
   */
  bool is_running() const { return running_; }

  /**
   * @brief Get the current go signal state.
   */
  bool get_go_signal() const { return go_signal_; }

  /**
   * @brief Get the current EBS state.
   */
  bool is_ebs_active() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    return ebs_active_;
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

  /**
   * @brief Get snapshot of current control input.
   * @return InputSnapshot Current throttle and steering values.
   */
  InputSnapshot get_input_snapshot() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    InputSnapshot snapshot;
    snapshot.throttle = throttle_;
    snapshot.steering = steering_;
    return snapshot;
  }

  /**
   * @brief Get the map information.
   * @return MapSnapshot Latest map snapshot.
   */
  MapSnapshot get_map_snapshot() const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    return map_snapshot_;
  }

  /**
   * @brief Get the latest sensors snapshot.
   * @return SensorsSnapshot Latest snapshot with sensors data.
   */
  SensorsSnapshot get_sensors_snapshot() const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    return sensors_snapshot_;
  }

  /**
   * @brief Get the latest vehicle state snapshot.
   * @return VehicleStateSnapshot Latest snapshot with vehicle state data.
   */
  VehicleStateSnapshot get_vehicle_state_snapshot() const {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    return vehicle_state_snapshot_;
  }

private:
  InvictaSimParameters params_;  ///< Simulator configuration values.

  // Simulation components
  std::shared_ptr<VehicleModel> vehicle_model_;  ///< Vehicle model.
  std::shared_ptr<Track> track_;                 ///< Track information.

  // Simulation loop timing
  std::atomic<bool> running_;  ///< Indicates whether the simulation loop is running.
  double sim_time_;            ///< Current simulation time in seconds.
  std::chrono::steady_clock::duration step_duration_;  ///< Duration of each simulation step.
  std::chrono::steady_clock::time_point
      next_step_time_;  ///< Next wall-clock time for a simulation step.
  std::chrono::steady_clock::time_point
      last_step_time_;  ///< Last wall-clock time used to compute step dt.

  // Output snapshot data
  mutable std::mutex output_snapshot_mutex_;         ///< Protects output snapshot access.
  VehicleModelSnapshot vehicle_model_snapshot_;      ///< Latest vehicle model snapshot.
  ExecutionTimesSnapshot execution_times_snapshot_;  ///< Latest per-step execution timings.
  MapSnapshot map_snapshot_;                         ///< Latest map snapshot.
  SensorsSnapshot sensors_snapshot_;                 ///< Latest sensors snapshot.
  VehicleStateSnapshot vehicle_state_snapshot_;      ///< Latest vehicle state snapshot.

  // Current commands
  mutable std::mutex input_mutex_;           ///< Protects input access.
  common_lib::structures::Wheels throttle_;  ///< Current throttle commands (all wheels).
  double steering_;                          ///< Current steering command (radians).
  bool ebs_active_{false};                   ///< Current EBS state.

  std::atomic<bool> go_signal_{false};       ///< Global go signal state.

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
   * @brief Build map snapshot with ground truth map, slam simulation and perception cones.
   * @return MapSnapshot Latest map snapshot.
   */
  MapSnapshot build_map_snapshot() const;

  /**
   * @brief Build sensors snapshot with latest simulated sensors data.
   * @return SensorsSnapshot Latest sensors snapshot.
   */
  SensorsSnapshot build_sensors_snapshot(const VehicleModelSnapshot& vehicle_snapshot) const;

  /**
   * @brief Build vehicle state snapshot with latest needed information for other subsystems.
   * @return VehicleStateSnapshot Latest vehicle state snapshot.
   */
  VehicleStateSnapshot build_vehicle_state_snapshot() const;

  /**
   * @brief Execute a simulation step.
   */
  void simulation_step();
};
