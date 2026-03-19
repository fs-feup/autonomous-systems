#pragma once

#include <ncurses.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>

#include "io/input/input_adapter.hpp"

/**
 * @brief Keyboard-based simulator input adapter.
 */
class KeyboardInputAdapter : public InvictaSimInputAdapter {
public:
  /**
   * @brief Construct a new KeyboardInputAdapter.
   * @param simulator Simulator instance.
   */
  explicit KeyboardInputAdapter(const std::shared_ptr<InvictaSim>& simulator);

  /**
   * @brief Destroy the KeyboardInputAdapter.
   */
  ~KeyboardInputAdapter() override;

  /**
   * @brief Start reading keyboard input.
   */
  void run() override;

  /**
   * @brief Stop reading keyboard input.
   */
  void stop() override;

private:
  /**
   * @brief Run the keyboard polling loop.s
   */
  void input_loop();

  /**
   * @brief Restore terminal state before exit.
   */
  void restore_terminal();

  const int loop_period_ms_;              ///< Keyboard polling period in milliseconds.
  const double acceleration_step_;        ///< Step applied when increasing throttle.
  const double acceleration_decay_step_;  ///< Decay applied when no throttle key is pressed.
  const double steering_step_radians_;    ///< Step applied when changing steering.
  const double max_steering_radians_;     ///< Maximum steering magnitude.
  const int help_row_;                    ///< Row used to print help text.
  const int status_row_;                  ///< Row used to print live status.

  std::atomic<bool> running_;  ///< Indicates whether the adapter loop is active.
  bool curses_initialized_;    ///< Indicates whether curses was initialized.
  FILE* tty_file_;             ///< TTY handle used by curses.
  void* screen_;               ///< Native curses screen handle.
};
