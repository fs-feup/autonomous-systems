#pragma once

#include <atomic>
#include <chrono>
#include <memory>

#include "config/config.hpp"
#include "io/input/input_adapter.hpp"
#include "io/output/output_adapter.hpp"
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
   * @param input_adapter Input adapter.
   * @param output_adapter Output adapter.
   */
  InvictaSim(const InvictaSimParameters& params,
             const std::shared_ptr<InvictaSimInputAdapter>& input_adapter,
             const std::shared_ptr<InvictaSimOutputAdapter>& output_adapter);

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

private:
  InvictaSimParameters params_;                              ///< Simulator configuration values.
  std::shared_ptr<VehicleModel> vehicle_model_;              ///< Vehicle model.
  std::shared_ptr<InvictaSimInputAdapter> input_adapter_;    ///< Simulator input adapter.
  std::shared_ptr<InvictaSimOutputAdapter> output_adapter_;  ///< Simulator output adapter.
  std::atomic<bool> running_;  ///< Indicates whether the simulation loop is running.
  double sim_time_;            ///< Current simulation time in seconds.
  std::chrono::steady_clock::time_point
      next_loop_time_;  ///< Next wall-clock time for a simulation step.

  /**
   * @brief Execute a simulation step.
   */
  void simulation_step();
};
