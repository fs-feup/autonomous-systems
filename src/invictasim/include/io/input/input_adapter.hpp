#pragma once

#include "common_lib/structures/wheels.hpp"

/**
 * @brief Input message that the simulator receives.
 */
struct InvictaSimInput {
  common_lib::structures::Wheels throttle = {0.0, 0.0, 0.0, 0.0};  ///< Wheel throttle commands.
  double steering = 0.0;                                           ///< Steering command.
};

/**
 * @brief Base interface for simulator input adapters.
 */
class InvictaSimInputAdapter {
public:
  /**
   * @brief Destroy the input adapter.
   */
  virtual ~InvictaSimInputAdapter() = default;

  /**
   * @brief Start the adapter loop, if needed (e.g., for keyboard input).
   */
  virtual void run() {}

  /**
   * @brief Stop the adapter loop, if needed.
   */
  virtual void stop() {}

  /**
   * @brief Get the latest input values.
   * @return Current simulator input.
   */
  virtual InvictaSimInput get_current_input() const = 0;
};
