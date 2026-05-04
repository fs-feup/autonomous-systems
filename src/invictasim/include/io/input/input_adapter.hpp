#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "simulator/invictasim.hpp"

/**
 * @brief Base interface for simulator input adapters.
 */
class InvictaSimInputAdapter {
public:
  /**
   * @brief Construct a new input adapter and start the global terminal listener.
   * @param simulator Simulator instance.
   */
  explicit InvictaSimInputAdapter(const std::shared_ptr<InvictaSim>& simulator);

  /**
   * @brief Destroy the input adapter and stop the terminal listener.
   */
  virtual ~InvictaSimInputAdapter();

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

private:
  std::thread terminal_thread_;                ///< Thread for global terminal listener.
  std::atomic<bool> terminal_running_{false};  ///< Flag to control terminal listener loop.

  /**
   * @brief Background loop to listen for global terminal hotkeys.
   */
  void terminal_loop();
};
