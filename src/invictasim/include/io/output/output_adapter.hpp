#pragma once

#include <memory>

#include "simulator/invictasim.hpp"
#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Base interface for simulator output adapters.
 */
class InvictaSimOutputAdapter {
public:
  /**
   * @brief Construct a new output adapter.
   * @param simulator Simulator instance.
   */
  explicit InvictaSimOutputAdapter(const std::shared_ptr<InvictaSim>& simulator)
      : simulator_(simulator) {}

  /**
   * @brief Destroy the output adapter.
   */
  virtual ~InvictaSimOutputAdapter() = default;

  /**
   * @brief Start the adapter loop if needed.
   */
  virtual void run() {}

  /**
   * @brief Stop the adapter loop if needed.
   */
  virtual void stop() {}

  /**
   * @brief Publish method for the simulator class.
   * @param vehicle_model Vehicle model with the current state.
   * @param steering_angle Current steering command.
   */
  virtual void publish_outputs(const std::shared_ptr<VehicleModel>& vehicle_model,
                               double steering_angle) {}

protected:
  std::shared_ptr<InvictaSim> simulator_;  ///< Reference to simulator instance.
};
