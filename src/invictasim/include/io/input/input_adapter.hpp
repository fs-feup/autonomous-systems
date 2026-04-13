#pragma once

#include <memory>

#include "simulator/invictasim.hpp"

/**
 * @brief Base interface for simulator input adapters.
 */
class InvictaSimInputAdapter {
public:
  /**
   * @brief Construct a new input adapter.
   * @param simulator Simulator instance.
   */
  explicit InvictaSimInputAdapter(const std::shared_ptr<InvictaSim>& simulator)
      : simulator_(simulator) {}

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

protected:
  std::shared_ptr<InvictaSim> simulator_;  ///< Reference to simulator instance.
};
